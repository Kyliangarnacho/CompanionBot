"""Stage 8 Final dataflow demo/acceptance: real Qwen, fake execution, no GUI.

All natural prompts are synthetic acceptance fixtures. Raw successes/failures,
tool IDs, provider usage and async task events stay under Stage 8 results.
"""
from __future__ import annotations
import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault('PYDANTIC_AI_NO_BANNER', '1')

from embodied_agent.behavior import BehaviorSupervisor, FakeNavigationBackend, FakeRobotBackend
from embodied_agent.catalog import CatalogQuery, CatalogResolver, JsonCatalogProvider, json_object
from embodied_agent.config import AgentConfig, STAGE_DIR
from embodied_agent.interaction import AgentSession
from embodied_agent.runtime import qwen_runtime


async def main(args):
    config = AgentConfig.load()
    provider = JsonCatalogProvider.load(STAGE_DIR / 'config/knowledge.json')
    nav = FakeNavigationBackend(provider.destination_ids(), cancel_immediate=False)
    supervisor = BehaviorSupervisor(FakeRobotBackend(), nav)
    directory = STAGE_DIR / 'results' / ('stage82_qwen_' + uuid4().hex)
    directory.mkdir(parents=True)
    runtime = qwen_runtime(supervisor, config, catalog=provider, telemetry=directory / 'requests.jsonl',
        master_status=lambda: {'available':True,'visible':True,'state':'LOCKED','provider':'synthetic_acceptance_fixture',
                              'hardware_execution_ready':False})
    session = AgentSession(runtime, preempt_on_input=True)
    runtime.trace_fixture_tool_args = True  # Synthetic fixtures only; production omits user query text.
    records, tool_events = [], []
    runtime.on_event = lambda kind, data: tool_events.append({'kind':kind, **data})

    async def turn(name, prompt, *, expected_destination=None, expected_catalog=None, expect_action=None,
                   expected_entity=None, ablate=False, expansion_required=False, original_query=None):
        start_events = len(tool_events)
        await session.receive(prompt)
        outcome = await session.wait()
        events = tool_events[start_events:]
        searches = [e for e in events if e['kind']=='catalog']
        accepted = [e for e in outcome.metrics['behavior_events'] if e['status']=='ACCEPT'] if outcome else []
        valid = bool(outcome and outcome.status=='COMPLETED')
        if outcome and expect_action is False and outcome.metrics.get('answer_source')=='local_catalog_fallback':
            valid = True  # Expected refusal/clarification, with model failure retained in metrics.
        if expected_destination:
            valid &= any(e['destination_id']==expected_destination for e in accepted)
        if expected_catalog:
            valid &= any(e['status']==expected_catalog for e in searches)
        if expect_action is False:
            valid &= not accepted
        if expect_action == 'FOLLOW':
            valid &= any(e['intent']=='FOLLOW' for e in accepted)
        first_call = next((e for e in events if e['kind']=='fixture_tool_args' and e['name']=='lookup_knowledge'), None)
        first_result = next((e['result'] for e in events if e['kind']=='fixture_catalog_result'), None)
        if expected_entity:
            valid &= bool(first_result and {i['entity']['id'] for i in first_result['items']} == {expected_entity})
        comparison = None
        first_validation_error = None
        batch = None
        if ablate and first_call and not first_call['args'].get('refine_pending'):
            batch, search_call = None, None
            for call in (e for e in events if e['kind']=='fixture_tool_args' and e['name']=='lookup_knowledge'):
                params = dict(call['args'])
                params.pop('refine_pending', None)
                try:
                    if isinstance(params.get('attributes'), str): params['attributes'] = json_object(params['attributes'])
                    if params.get('attributes') is None: params.pop('attributes', None)
                    batch = CatalogQuery.model_validate(params)
                except ValueError as error:
                    if call is first_call: first_validation_error = type(error).__name__
                    continue
                search_call = call
                break
        if ablate and batch is not None:
            baseline_query = batch.model_copy(update={'query_variants':()})
            baseline = await CatalogResolver(provider).lookup(baseline_query, turn_id='ablation', input_id='fixture')
            comparison = {'design':'same_catalog_same_first_call_constraints_variants_off_no_extra_model_request',
                'catalog_revision':provider.revision,'query_without_variants':baseline_query.model_dump(mode='json'),
                'baseline_result':baseline,'expanded_result':first_result,
                'search_call_model_request_index':search_call['model_request_index']}
            if original_query:
                raw = baseline_query.model_copy(update={'query':original_query})
                comparison['user_expression_baseline'] = await CatalogResolver(provider).lookup(raw,
                    turn_id='raw_ablation', input_id='fixture')
        if expansion_required:
            valid &= bool(first_call and first_call['model_request_index']==1 and first_call['args'].get('query_variants')
                and comparison and comparison['baseline_result']['status']=='NO_MATCH' and first_result
                and first_result['status']=='RESOLVED' and original_query in first_call['args']['query'])
        record={'case':name,'fixture_prompt':prompt,'validated':bool(valid),
                'answer':outcome.text if outcome else None,'metrics':outcome.metrics if outcome else None,
                'tool_events':events, 'first_lookup_call':first_call, 'ablation':comparison,
                'first_lookup_validation_error':first_validation_error}
        records.append(record)
        (directory/'transcript.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),'utf-8')
        print(json.dumps({'case':name,'validated':valid,'answer':record['answer'],
                          'requests':outcome.metrics['requests'] if outcome else None,
                          'catalog':[e['status'] for e in searches],
                          'first_query':first_call['args'] if first_call else None,
                          'baseline_status':comparison['baseline_result']['status'] if comparison else None,
                          'destination_ids':[e['destination_id'] for e in accepted]},ensure_ascii=False),flush=True)
        return outcome

    async def settle():
        await supervisor.cancel_all()
        for task_id, result in list(nav.tasks.items()):
            if result.status=='CANCEL_REQUESTED':
                await supervisor.receive_feedback(nav.acknowledge_cancel(task_id))
        runtime.clear_history()

    try:
        await session.receive('领我去找快食面')
        assert not runtime.outcomes and runtime.network_counter['attempts']==0
        await session.receive(config.wake_phrase)
        if args.case in ('all','semantic'):
            await turn('expansion_colloquial','带我去找快食面。',expected_destination='instant_noodle_zone',
                expected_catalog='RESOLVED',ablate=True,expansion_required=True,original_query='快食面')
            await settle()
            await turn('expansion_function_description','我想找那种开水泡几分钟就能吃的面。',
                expected_catalog='RESOLVED',expect_action=False,ablate=True,expansion_required=True,
                original_query='开水泡几分钟就能吃的面')
            await settle()
            await turn('pasta_not_instant','我要找意大利面。',expected_catalog='RESOLVED',ablate=True)
            # Validate resolved category even when the user only asked to find it.
            if records[-1]['ablation']:
                resolution=records[-1]['ablation']['expanded_result'].get('resolution')
                records[-1]['validated'] &= bool(resolution and resolution['destination_id']=='pasta_zone')
            else: records[-1]['validated']=False
            await settle()
            await turn('brand_flavor_packaging_preserved','找康师傅红烧牛肉面袋装。',
                expected_catalog='RESOLVED',expected_entity='mk_beef_bag',ablate=True)
            await settle()
            await turn('same_brand_different_category','我想找康师傅的饮料。',
                expected_catalog='RESOLVED',expected_entity='mk_iced_tea',ablate=True)
            await settle()
            await turn('nonexistent_exact_sku','带我找 SKU MODEL-INVENTED-SKU。',
                expected_catalog='NO_MATCH',expect_action=False,ablate=True)
            await settle()
            await turn('real_ambiguity','带我买康师傅红烧牛肉面。',
                expected_catalog='CLARIFY',expect_action=False,ablate=True)
            await turn('supplement_after_expansion','袋装的。',expected_catalog='RESOLVED',
                expected_destination='instant_noodle_zone',expected_entity='mk_beef_bag')
            await settle()
            await turn('cross_destination_ambiguity','带我去买苹果汁。',
                expected_catalog='CLARIFY',expect_action=False,ablate=True)
            await turn('cross_destination_supplement','演示乙的，五百毫升。',
                expected_catalog='RESOLVED',expected_destination='drinks_b',expected_entity='apple_b')
            await settle()
            await turn('irrelevant_greeting','你好，今天心情不错。',expect_action=False)
            records[-1]['validated'] &= records[-1]['first_lookup_call'] is None
            await settle()
        if args.case in ('all','catalog'):
            await turn('colloquial_category','带我找一下快食面。',expected_destination='instant_noodle_zone',expected_catalog='RESOLVED')
            task=supervisor.active_task
            if task in nav.tasks:
                await supervisor.receive_feedback(nav.advance(task,.4))
                await asyncio.sleep(.02)
                await supervisor.receive_feedback(nav.complete(task))
                records.append({'case':'async_completion_after_model_end','validated':session.task.done() and
                    (await supervisor.status(task)).status=='COMPLETED','task_id':task})
            await settle()
            await turn('where_category','意大利面在哪里？',expected_catalog='RESOLVED',expect_action=False)
            await settle()
            await turn('colloquial_exhibit','领我过去看看那个机器人展品。',expected_destination='robot_exhibit')
            await settle()
            await turn('specific_packaging_ambiguity','我想买康师傅那个红烧牛肉味的，帮我领过去。',expected_catalog='CLARIFY',expect_action=False)
            await turn('followup_packaging','袋装的。',expected_destination='instant_noodle_zone',expected_catalog='RESOLVED')
            await settle()
            await turn('shared_area_many_products','我只想去摆红烧牛肉面的货架看看，不挑品牌和规格。',expected_destination='instant_noodle_zone')
            await settle()
            await turn('cross_area_brand_ambiguity','带我去买苹果汁。',expected_catalog='CLARIFY',expect_action=False)
            await turn('followup_brand_size','演示乙的，五百毫升。',expected_destination='drinks_b',expected_catalog='RESOLVED')
            await settle()
            await turn('no_match','领我去找量子芒果干。',expected_catalog='NO_MATCH',expect_action=False)
            await settle()
            await turn('specific_sku','带我去找 SKU MK-BEEF-BAG-103 那款。',expected_destination='instant_noodle_zone')
            await settle()
            await turn('unknown_sku','带我找 SKU MODEL-INVENTED-SKU。',expected_catalog='NO_MATCH',expect_action=False)
            await settle()
            await turn('expired_pending_start','带我买康师傅红烧牛肉面。',expected_catalog='CLARIFY',expect_action=False)
            if runtime.resolver.pending:
                runtime.resolver.pending.expires_at_s = time.perf_counter()-1
            await turn('expired_pending_supplement','袋装的。',expect_action=False)
            await settle()
        if args.case in ('all','controls'):
            await turn('semantic_follow','小柒，在我后面陪我走一段。',expect_action='FOLLOW')
            await turn('semantic_cancel','这趟不用去了，把刚才的任务撤掉。')
            records.append({'case':'semantic_cancel_terminal','validated':supervisor.active_task is None})
            await settle()
            await turn('cancel_pending_guide','陪我去咨询台看看。',expected_destination='service_desk')
            task=supervisor.active_task
            cancelled=await supervisor.cancel()
            if task in nav.tasks:
                first=await supervisor.status(task)
                completed=await supervisor.receive_feedback(nav.acknowledge_cancel(task))
                records.append({'case':'cancel_request_then_ack','validated':cancelled.status=='CANCEL_REQUESTED'
                    and first.status=='CANCEL_REQUESTED' and completed.status=='CANCEL','task_id':task})
            await settle()
        if args.case in ('all','privacy'):
            await turn('semantic_sleep','你退下吧。')
            before=runtime.network_counter['attempts']
            if session.state.value=='SLEEP':
                await session.receive('帮我看看当前画面',vision=True)
            records.append({'case':'sleep_zero_requests','validated':session.state.value=='SLEEP' and
                            runtime.network_counter['attempts']==before})
            await session.receive('/sleep')
            before=runtime.network_counter['attempts']
            await session.receive('帮我看看当前画面',vision=True)
            records.append({'case':'local_sleep_zero_requests','validated':session.state.value=='SLEEP' and
                            runtime.network_counter['attempts']==before})
    finally:
        await settle()
        await session.close()
        (directory/'transcript.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),'utf-8')
        summary={'model':config.model,'catalog_revision':provider.revision,'validated':bool(records) and all(r['validated'] for r in records),
            'network_attempts':runtime.network_counter['attempts'],'input_tokens':sum(r['metrics']['input_tokens'] for r in records if r.get('metrics')),
            'output_tokens':sum(r['metrics']['output_tokens'] for r in records if r.get('metrics')),
            'cases':[{'case':r['case'],'validated':r['validated']} for r in records], 'backend':'fake',
            'model_failures':sum(r['metrics']['status']=='FAILED' for r in records if r.get('metrics')),
            'catalog_fallbacks':sum(r['metrics'].get('answer_source')=='local_catalog_fallback' for r in records if r.get('metrics')),
            'master':'synthetic_acceptance_fixture','gui_opened':False}
        summary['case_selection']=args.case
        summary['request_limit']=config.request_limit
        summary['wall_s']=sum(r['metrics']['wall_s'] for r in records if r.get('metrics'))
        summary['ablation_expansion_gains']=[r['case'] for r in records if r.get('ablation') and
            r['ablation']['baseline_result']['status']=='NO_MATCH' and
            r['ablation']['expanded_result']['status']=='RESOLVED']
        (directory/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),'utf-8')
        print('Saved: '+str(directory),flush=True)
    return summary['validated']


if __name__=='__main__':
    for stream in (sys.stdout,sys.stderr):
        if hasattr(stream,'reconfigure'): stream.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['all','catalog','controls','privacy','semantic'],default='all')
    try:
        sys.exit(0 if asyncio.run(main(parser.parse_args())) else 1)
    except Exception as error:
        print('Verification failed: '+type(error).__name__,file=sys.stderr)
        sys.exit(1)
