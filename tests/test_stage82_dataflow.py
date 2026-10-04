"""Contract and adversarial lifecycle tests; semantic coverage uses live Qwen separately."""
import asyncio
from dataclasses import replace
import json
import time

import numpy as np
import pytest
from pydantic import ValidationError
from pydantic_ai.models.function import DeltaToolCall, FunctionModel

from embodied_agent.behavior import BehaviorRequest, BehaviorSupervisor, FakeNavigationBackend, FakeRobotBackend, Feedback
from embodied_agent.catalog import Candidate, CatalogDocument, CatalogQuery, CatalogResolver, JsonCatalogProvider
from embodied_agent.catalog import json_object
from embodied_agent.config import AgentConfig, STAGE_DIR
from embodied_agent.frames import FrameROI, Stage7FrameBuffer, encode_keyframe
from embodied_agent.inputs import ASRMetadata
from embodied_agent.interaction import AgentSession
from embodied_agent.runtime import AgentRuntime
from perception.camera import ColorFrame


def catalog():
    return JsonCatalogProvider.load(STAGE_DIR / 'config/knowledge.json')


def test_expansion_union_provenance_and_alias_free_ablation():
    async def run():
        provider = catalog()
        assert not any('快食面' in r.aliases for r in provider.document.categories)
        for text in ['快食面', '开水泡几分钟就能吃的面']:
            old = await provider.search(CatalogQuery(query=text, target_kind='category'))
            assert not old.items and not old.categories
            new = await provider.search(CatalogQuery(query=text, query_variants=('方便面', '方便面', '速食面'), target_kind='category'))
            assert [c.id for c in new.categories] == ['instant_noodles']
            assert len({c.entity.id for c in new.items}) == len(new.items) == 4
            assert new.queries == (text, '方便面', '速食面')
            assert all(c.matches[0].query_index == 1 for c in new.items)
            assert all(c.matches[0].field == 'category' for c in new.items)
            assert new.queries[new.matches[0].query_index] == '方便面'
    asyncio.run(run())


def test_conflicting_hypotheses_are_not_resolved_by_highest_score():
    async def run():
        resolver = CatalogResolver(catalog())
        first = await resolver.lookup(CatalogQuery(query='想找能吃的面', query_variants=('方便面', '意大利面'),
            target_kind='category'), turn_id='1', input_id='i1')
        assert first['status'] == 'CLARIFY' and first['resolution'] is None
        assert {c['id'] for c in first['categories']} == {'instant_noodles', 'pasta'}
        assert len(first['items']) == 6
        refined = await resolver.lookup(CatalogQuery(query='意大利面'), turn_id='2', input_id='i2', refine_pending=True)
        assert refined['resolution']['destination_id'] == 'pasta_zone'
        assert refined['queries'] == ['意大利面']  # Old hypotheses cannot leak into supplement.
        await resolver.lookup(CatalogQuery(query='未知描述', query_variants=('方便面', '百味来斜管面'),
            target_kind='category'), turn_id='3', input_id='i3')
        assert resolver.pending and not resolver.resolution
        rejected = await resolver.lookup(CatalogQuery(query='服务台'), turn_id='4', input_id='i4', refine_pending=True)
        assert rejected['resolution'] is None  # A supplement cannot jump to unrelated region.
    asyncio.run(run())


@pytest.mark.parametrize('query,sku', [('MODEL-INVENTED-SKU', None), ('红烧牛肉面', 'MODEL-INVENTED-SKU')])
def test_expansions_cannot_replace_exact_identifier(query, sku):
    async def run():
        resolver = CatalogResolver(catalog())
        result = await resolver.lookup(CatalogQuery(query=query, sku=sku,
            query_variants=('方便面', 'MK-BEEF-BAG-103')), turn_id='t', input_id='i')
        assert result['status'] == 'NO_MATCH' and result['resolution'] is None
        assert result['queries'] == [query]
        assert result['ignored_query_variants_reason'] == 'exact_identifier_constraint'
    asyncio.run(run())


def test_expansion_filters_remain_strict_and_do_not_cross_brand_category():
    async def run():
        provider = catalog()
        result = await provider.search(CatalogQuery(query='功能描述', query_variants=('红烧牛肉面', '饮料'),
            brand='康师傅', category='方便面', attributes={'flavor':'红烧牛肉', 'packaging':'袋装', 'size':'103g'}))
        assert [c.entity.id for c in result.items] == ['mk_beef_bag']
        drink = await provider.search(CatalogQuery(query='饮料', brand='康师傅', query_variants=('茶饮料',)))
        assert [c.entity.id for c in drink.items] == ['mk_iced_tea']
        missing = await provider.search(CatalogQuery(query='功能描述', query_variants=('方便面',), brand='不存在的品牌'))
        assert not missing.items
        spaced = await CatalogResolver(provider).lookup(CatalogQuery(query='康师傅红烧牛肉面袋装',
            query_variants=('康师傅 红烧牛肉面 袋装',), brand='康师傅',
            attributes={'口味':'红烧牛肉','包装':'袋装'}), turn_id='t', input_id='i')
        assert spaced['status']=='RESOLVED' and spaced['resolution']['entity_ids']==('mk_beef_bag',)
        assert len(spaced['queries']) == 2
        full = await provider.search(CatalogQuery(query='MK-BEEF-BAG-103'))
        assert Candidate.model_validate(spaced['items'][0]).entity == full.items[0].entity
        joined = await CatalogResolver(provider).lookup(CatalogQuery(query='康师傅红烧牛肉面袋装',
            query_variants=('康师傅 红烧牛肉 袋装方便面',), brand='康师傅',
            attributes={'flavor':'红烧牛肉','packaging':'袋装'}), turn_id='t', input_id='i')
        assert joined['status'] == 'RESOLVED' and joined['resolution']['entity_ids']==('mk_beef_bag',)
        apple = await CatalogResolver(provider).lookup(CatalogQuery(query='苹果汁', query_variants=('果汁',),
            target_kind='product'), turn_id='t', input_id='i')
        assert apple['status']=='CLARIFY' and apple['resolution'] is None
    asyncio.run(run())


def test_query_batch_schema_bounds_and_duplicate_match_evidence():
    for patch in [{'query_variants':['a','b','c','d']}, {'query_variants':['']}, {'query_variants':[' '*5]},
                  {'query_variants':['x'*161]}]:
        with pytest.raises(ValidationError): CatalogQuery(query='描述', **patch)
    runtime = AgentRuntime(FunctionModel(stream_function=lambda *_: None), BehaviorSupervisor(
        FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig())
    schema = runtime.agent._function_toolset.tools['lookup_knowledge'].function_schema
    assert schema.json_schema['properties']['query_variants']['maxItems'] == 3
    assert schema.validator.validate_python({'query':'名字'})['query_variants'] == ()
    with pytest.raises(ValidationError): schema.validator.validate_python({'query':'名字','query_variants':['a']*4})
    async def run():
        result = await catalog().search(CatalogQuery(query='MK-BEEF-BAG-103', query_variants=('方便面',)))
        assert len(result.items) == 1
        assert result.ignored_query_variants_reason
        union = await catalog().search(CatalogQuery(query='红烧牛肉面', query_variants=('康师傅红烧牛肉面',)))
        bag = next(c for c in union.items if c.entity.id == 'mk_beef_bag')
        assert len(bag.matches) == 2 and len({c.entity.id for c in union.items}) == len(union.items)
    asyncio.run(run())


def test_expansion_truncation_and_non_json_provider_contract():
    async def run():
        provider = catalog()
        data = provider.document.model_dump()
        data['items'] = [dict(id=f'x{i}', name='同名', destination_id='service_desk') for i in range(35)]
        provider = JsonCatalogProvider(CatalogDocument.model_validate(data))
        result = await CatalogResolver(provider).lookup(CatalogQuery(query='未知描述', query_variants=('同名',),
            target_kind='area'), turn_id='t', input_id='i')
        assert result['status'] == 'CLARIFY' and result['truncated'] and result['total_count'] == 35
        assert len(result['items']) == 30
        # Resolver depends on typed search and validation only, never a JSON document.
        class Remote:
            revision = provider.revision
            async def search(self, query): return await provider.search(query)
            def destination_valid(self, value): return provider.destination_valid(value)
            def entity_valid(self, value): return provider.entity_valid(value)
            def category_valid(self, value): return provider.category_valid(value)
        remote = await CatalogResolver(Remote()).lookup(CatalogQuery(query='未知描述', query_variants=('同名',)),
            turn_id='t', input_id='i')
        assert remote['truncated'] and remote['resolution'] is None
    asyncio.run(run())


def test_large_category_resolves_without_hiding_truncated_expansion_conflict():
    async def run():
        data = catalog().document.model_dump()
        template = data['items'][3]
        data['items'] = [dict(template, id=f'a{i:03}', sku=f'FIXTURE-{i}') for i in range(100)]
        provider = JsonCatalogProvider(CatalogDocument.model_validate(data))
        result = await CatalogResolver(provider).lookup(CatalogQuery(query='快食面', query_variants=('方便面',),
            target_kind='category'), turn_id='t', input_id='i')
        assert result['truncated'] and len(result['items'])==30 and result['total_count']==100
        assert result['status']=='RESOLVED' and result['resolution']['destination_id']=='instant_noodle_zone'
        # An entity hidden beyond the returned 30 must still prevent a unique region.
        data['items'].append(dict(id='zz_other', name='纪念杯', destination_id='gift_zone'))
        provider = JsonCatalogProvider(CatalogDocument.model_validate(data))
        result = await CatalogResolver(provider).lookup(CatalogQuery(query='快食面', query_variants=('方便面','纪念杯'),
            target_kind='category'), turn_id='t', input_id='i')
        assert result['truncated'] and result['total_count']==101
        assert 'zz_other' not in {c['entity']['id'] for c in result['items']}
        assert result['region_entity_conflict'] and result['status']=='CLARIFY' and result['resolution'] is None
    asyncio.run(run())


@pytest.mark.parametrize('query,kind,destination,status', [
    ('方便面','auto','instant_noodle_zone','RESOLVED'),
    ('意大利面','category','pasta_zone','RESOLVED'),
    ('机器人展品','area','robot_exhibit','RESOLVED'),
    ('咨询台','destination','service_desk','RESOLVED'),
    ('红烧牛肉面','product',None,'CLARIFY'),
    ('红烧牛肉面','area','instant_noodle_zone','RESOLVED'),
    ('苹果汁','area',None,'CLARIFY'),
    ('量子芒果干','auto',None,'NO_MATCH'),
    ('纪念杯','product','gift_zone','RESOLVED'),
    ('果汁','category',None,'UNAVAILABLE'),
    ('mk_beef_bad','product',None,'NO_MATCH'),
])
def test_data_driven_resolution(query, kind, destination, status):
    async def run():
        resolver = CatalogResolver(catalog())
        result = await resolver.lookup(CatalogQuery(query=query, target_kind=kind), turn_id='t', input_id='i')
        assert result['status'] == status, result
        if destination:
            r = result['resolution']
            assert r['destination_id'] == destination and resolver.valid(r['resolution_id'], destination, 't')
            assert not resolver.valid(r['resolution_id'], destination, 'old-turn')
        else:
            assert result['resolution'] is None
    asyncio.run(run())


def test_brand_flavor_packaging_refinement_and_fabricated_sku():
    async def run():
        resolver = CatalogResolver(catalog())
        first = await resolver.lookup(CatalogQuery(query='红烧牛肉', target_kind='product', brand='康师傅'), turn_id='1', input_id='i1')
        assert first['status'] == 'CLARIFY' and len(first['items']) == 2
        refined = await resolver.lookup(CatalogQuery(query='袋装', attributes={'包装':'袋装'}), turn_id='2', input_id='i2', refine_pending=True)
        assert refined['status'] == 'RESOLVED' and refined['resolution']['entity_ids'] == ('mk_beef_bag',)
        missing = await resolver.lookup(CatalogQuery(query='红烧牛肉', sku='MODEL-INVENTED-SKU'), turn_id='3', input_id='i3')
        assert missing['status'] == 'NO_MATCH' and resolver.resolution is None
    asyncio.run(run())


@pytest.mark.parametrize('invalidate', ['ttl','revision','clear'])
def test_candidate_lifecycle_invalidates_instead_of_reusing(invalidate):
    async def run():
        now = [10.0]
        provider = catalog()
        resolver = CatalogResolver(provider, clock=lambda: now[0], ttl_s=2)
        await resolver.lookup(CatalogQuery(query='红烧牛肉面', target_kind='product'), turn_id='1', input_id='i1')
        if invalidate == 'ttl': now[0] += 3
        elif invalidate == 'revision': provider.document = provider.document.model_copy(update={'notice':'changed'})
        else: resolver.clear()
        result = await resolver.lookup(CatalogQuery(query='袋装'), turn_id='2', input_id='i2', refine_pending=True)
        assert result['status'] == 'REJECT' and resolver.pending is None and resolver.resolution is None
    asyncio.run(run())


def test_truncated_results_are_never_mistaken_for_unique():
    data = catalog().document.model_dump()
    data['items'] = [dict(id=f'x{i}', name='同名', destination_id='service_desk') for i in range(35)]
    async def run():
        resolver = CatalogResolver(JsonCatalogProvider(CatalogDocument.model_validate(data)))
        result = await resolver.lookup(CatalogQuery(query='同名', target_kind='area'), turn_id='t', input_id='i')
        assert result['status'] == 'CLARIFY' and result['truncated'] and result['total_count'] == 35
    asyncio.run(run())


def test_schema_invalid_references_and_v1_adapter_does_not_invent_item_destination():
    data = catalog().document.model_dump()
    data['items'][0]['destination_id'] = 'not-a-real-destination'
    with pytest.raises(ValidationError): CatalogDocument.model_validate(data)
    legacy = JsonCatalogProvider.from_dict({'notice':'demo','destinations':{'desk':'台'}, 'items':[{'id':'cup','name':'杯'}]})
    assert legacy.document.items[0].destination_id is None
    assert BehaviorRequest(intent='GUIDE_TO', destination='desk').model_dump()['destination_id'] == 'desk'
    with pytest.raises(ValidationError): BehaviorRequest(intent='GUIDE_TO', destination='desk', destination_id='other')


def test_qwen_nested_json_decoding_retains_typed_validation_and_flat_tool_schema():
    runtime=AgentRuntime(FunctionModel(stream_function=lambda *_: None),BehaviorSupervisor(
        FakeRobotBackend(),FakeNavigationBackend(set())),AgentConfig())
    schema=runtime.agent._function_toolset.tools['request_behavior'].function_schema
    assert 'intent' in schema.json_schema['properties'] and schema.single_arg_name=='request'
    for args in [{'intent':'FOLLOW'},{'request':{'intent':'FOLLOW'}},{'request':'{"intent":"FOLLOW"}'}]:
        assert schema.validator.validate_python(args)['request'].intent.value=='FOLLOW'
    with pytest.raises(ValidationError): schema.validator.validate_python({'intent':'PWM'})
    with pytest.raises(ValidationError): schema.validator.validate_python({'request':'{"intent":"FOLLOW","torque":1}'})
    lookup=runtime.agent._function_toolset.tools['lookup_knowledge'].function_schema
    parsed=lookup.validator.validate_python({'query':'红烧牛肉面','attributes':'{"包装":"袋装"}'})
    assert parsed['attributes']=={'包装':'袋装'}
    for invalid in ['[]','1','null','not-json']:
        with pytest.raises(ValueError): json_object(invalid)


def test_fuzzy_name_needs_confirmation_but_ids_and_skus_are_never_fuzzy():
    async def run():
        resolver=CatalogResolver(catalog())
        named=await resolver.lookup(CatalogQuery(query='康师傅香菇炖鸡面面',target_kind='product'),turn_id='1',input_id='1')
        # Near names can be recalled, but only confident names resolve; exact
        # reference-looking inputs must never resolve a neighboring identifier.
        assert named['items']
        for text in ['MK-BEEF-BAG-103X','mk_beef_bag_x','robot_exhibit_wrong']:
            result=await resolver.lookup(CatalogQuery(query=text,target_kind='product'),turn_id='2',input_id='2')
            assert result['status']=='NO_MATCH' and result['resolution'] is None
    asyncio.run(run())


def test_attribute_synonyms_are_supplied_by_data_and_unknown_fields_fail_closed():
    async def run():
        resolver=CatalogResolver(catalog())
        query=CatalogQuery(query='红烧牛肉面',target_kind='product',attributes={
            'brand':'Master Kong','flavor':'红烧牛肉味','packaging':'袋装','size':'103克'})
        result=await resolver.lookup(query,turn_id='t',input_id='i')
        assert result['resolution']['entity_ids']==('mk_beef_bag',)
        result=await resolver.lookup(CatalogQuery(query='红烧牛肉面',attributes={'invented':'xyz'}),turn_id='t',input_id='i')
        assert result['status']=='NO_MATCH'
    asyncio.run(run())


def test_nested_data_change_invalidates_a_previous_resolution():
    async def run():
        provider=catalog()
        resolver=CatalogResolver(provider)
        result=await resolver.lookup(CatalogQuery(query='MK-BEEF-BAG-103'),turn_id='t',input_id='i')
        r=result['resolution']
        assert resolver.valid(r['resolution_id'],r['destination_id'],'t')
        provider.document.items[3].attributes['包装']='changed'
        assert not resolver.valid(r['resolution_id'],r['destination_id'],'t')
    asyncio.run(run())


def test_ui_bridge_backlog_is_bounded_without_opening_gui():
    from embodied_agent.ui import DemoBridge
    bridge=DemoBridge()
    for i in range(2000):
        bridge.emit('stream',str(i))
        bridge.submit(str(i))
    assert bridge.commands.qsize()==64 and bridge.updates.qsize()==512
    assert list(bridge.commands.queue)[-1]=='1999'


def test_robot_foreign_clock_and_replayed_state_are_denied():
    async def run():
        robot=FakeRobotBackend()
        supervisor=BehaviorSupervisor(robot,FakeNavigationBackend(set()))
        state=await robot.latest_state()
        async def same(): return state
        robot.latest_state=same
        first=await supervisor.submit(BehaviorRequest(intent='FOLLOW'))
        assert first.status=='ACCEPT'
        await supervisor.cancel()
        assert (await supervisor.submit(BehaviorRequest(intent='FOLLOW'))).reason=='duplicate_or_out_of_order_robot_state'
        async def foreign(): return state.model_copy(update={'clock_domain':'ros_time','sequence':state.sequence+1})
        robot.latest_state=foreign
        assert (await supervisor.submit(BehaviorRequest(intent='FOLLOW'))).reason=='robot_clock_unmapped'
    asyncio.run(run())


def test_framework_semantic_tool_availability_and_current_turn_guide_receipt():
    async def run():
        provider = catalog()
        nav = FakeNavigationBackend(provider.destination_ids())
        supervisor = BehaviorSupervisor(FakeRobotBackend(), nav)
        seen = []
        async def stream(messages, info):
            seen.append({t.name for t in info.function_tools})
            returns = [p for p in messages[-1].parts if p.part_kind == 'tool-return']
            if returns and returns[0].tool_name == 'lookup_knowledge':
                r = returns[0].content['resolution']
                yield {0:DeltaToolCall(name='request_behavior',tool_call_id='guide',json_args=json.dumps({'request':{
                    'intent':'GUIDE_TO','destination_id':r['destination_id'],'resolution_id':r['resolution_id']}}))}
            elif returns:
                yield '模拟后端已接收请求。'
            else:
                yield {0:DeltaToolCall(name='lookup_knowledge',tool_call_id='search',json_args='{"query":"快食面","query_variants":["方便面"],"target_kind":"category"}')}
        runtime = AgentRuntime(FunctionModel(stream_function=stream), supervisor, AgentConfig(), catalog=provider)
        session = AgentSession(runtime, strict_behavior_intent=True)
        await session.receive('你好小柒')
        await session.receive('陪我过去瞧瞧卖面食的地方', input_id='human-input')
        result = await session.wait()
        assert result.status == 'COMPLETED' and result.metrics['requests'] == 3
        assert 'request_behavior' in seen[0] and 'request_behavior' not in seen[-1]
        event = next(r for r in result.metrics['behavior_events'] if r['status'] == 'ACCEPT')
        assert event['input_id'] == 'human-input' and event['turn_id'] == result.metrics['turn_id']
        assert event['destination_id'] == 'instant_noodle_zone'
        task_id = event['task_id']
        assert session.task.done() and (await supervisor.status(task_id)).status == 'ACCEPT'
        await supervisor.receive_feedback(nav.advance(task_id, .6))
        assert (await supervisor.status(task_id)).status == 'RUNNING'
        await supervisor.receive_feedback(nav.complete(task_id))
        assert supervisor.active_task is None and (await supervisor.status(task_id)).status == 'COMPLETED'
        await session.close()
    asyncio.run(run())


@pytest.mark.parametrize('destination,resolution', [('service_desk','invented'),('not-real','invented'),('robot_exhibit',None)])
def test_model_invented_or_unresolved_guide_never_dispatches(destination,resolution):
    async def run():
        provider = catalog()
        nav = FakeNavigationBackend(provider.destination_ids())
        supervisor = BehaviorSupervisor(FakeRobotBackend(), nav)
        async def stream(messages,info):
            if any(p.part_kind=='tool-return' for p in messages[-1].parts): yield '请求被拒绝。'
            else: yield {0:DeltaToolCall(name='request_behavior',tool_call_id='bad',json_args=json.dumps({'request':{
                'intent':'GUIDE_TO','destination_id':destination,'resolution_id':resolution}}))}
        runtime = AgentRuntime(FunctionModel(stream_function=stream), supervisor, AgentConfig(),catalog=provider)
        session = AgentSession(runtime)
        await session.receive('你好小柒请领路')
        result = await session.wait()
        assert not nav.tasks and supervisor.active_task is None
        assert result.metrics['behavior_events'][0]['reason'] == 'unresolved_or_expired_destination'
        await session.close()
    asyncio.run(run())


def test_failed_model_explanation_keeps_a_grounded_catalog_refusal_and_failure_evidence():
    async def run():
        provider=catalog()
        nav=FakeNavigationBackend(provider.destination_ids())
        supervisor=BehaviorSupervisor(FakeRobotBackend(),nav)
        async def stream(messages,info):
            if any(p.part_kind=='tool-return' for p in messages[-1].parts):
                raise RuntimeError('synthetic_explanation_failure')
            yield {0:DeltaToolCall(name='lookup_knowledge',tool_call_id='none',json_args='{"query":"量子芒果干"}')}
        runtime=AgentRuntime(FunctionModel(stream_function=stream),supervisor,AgentConfig(),catalog=provider)
        session=AgentSession(runtime)
        delivered=[]
        session.on_text=delivered.append
        await session.receive('你好小柒去找量子芒果干')
        outcome=await session.wait()
        assert outcome.status=='FAILED' and outcome.metrics['error_type']=='RuntimeError'
        assert outcome.metrics['answer_source']=='local_catalog_fallback' and not outcome.metrics['usage_complete']
        assert '没有匹配' in outcome.text and delivered==[outcome.text]
        assert not nav.tasks
        await session.close()
    asyncio.run(run())


def test_cancel_ack_is_distinct_from_cancel_request_and_late_feedback_is_isolated():
    async def run():
        nav = FakeNavigationBackend({'service_desk'}, cancel_immediate=False)
        supervisor = BehaviorSupervisor(FakeRobotBackend(),nav)
        first = await supervisor.submit(BehaviorRequest(intent='GUIDE_TO',destination_id='service_desk'),request_id='r',turn_id='t',input_id='i')
        assert (await supervisor.cancel()).status == 'CANCEL_REQUESTED' and supervisor.active_task == first.task_id
        assert (await supervisor.submit(BehaviorRequest(intent='FOLLOW'))).reason == 'task_busy_cancel_first'
        await supervisor.receive_feedback(nav.acknowledge_cancel(first.task_id))
        second = await supervisor.submit(BehaviorRequest(intent='FOLLOW'))
        assert second.status == 'ACCEPT'
        assert (await supervisor.receive_feedback(first)).status == 'CANCEL'
        assert supervisor.active_task == second.task_id
        wrong = second.model_copy(update={'task_id':'bogus'})
        assert (await supervisor.receive_feedback(wrong)).reason == 'unknown_or_expired_task_feedback'
        await supervisor.cancel_all()
    asyncio.run(run())


def test_timeout_unknown_busy_and_idempotency():
    async def run():
        robot = FakeRobotBackend()
        supervisor = BehaviorSupervisor(robot,FakeNavigationBackend(set()),backend_timeout_s=.02)
        original = robot.request
        async def lose_ack(task_id,request):
            await original(task_id,request)
            await asyncio.Event().wait()
        robot.request = lose_ack
        first = await supervisor.submit(BehaviorRequest(intent='FOLLOW'),request_id='once')
        assert first.status == 'UNKNOWN' and len(robot.requests)==1
        assert (await supervisor.submit(BehaviorRequest(intent='FOLLOW'),request_id='once')).task_id==first.task_id
        assert len(robot.requests)==1
        assert (await supervisor.submit(BehaviorRequest(intent='WAIT'),request_id='once')).reason=='duplicate_request_conflict'
        await supervisor.poll()  # Reconcile the known task via status, no replay.
        assert (await supervisor.status()).status=='ACCEPT'
        assert (await supervisor.submit(BehaviorRequest(intent='FOLLOW'))).reason=='task_busy_cancel_first'
        await supervisor.cancel_all()
    asyncio.run(run())


def test_task_deadline_and_bounded_ledgers():
    async def run():
        now=[10.0]
        robot=FakeRobotBackend(lambda:now[0])
        supervisor=BehaviorSupervisor(robot,FakeNavigationBackend(set()),clock=lambda:now[0],task_timeout_s=2,max_tasks=8)
        first=await supervisor.submit(BehaviorRequest(intent='FOLLOW'))
        now[0]+=3
        await supervisor.poll()
        assert (await supervisor.status(first.task_id)).status=='CANCEL'
        assert any(e.reason=='task_deadline_expired' for e in supervisor.events)
        for i in range(280):
            await supervisor.submit(BehaviorRequest(intent='WAIT'),request_id=str(i))
        assert len(supervisor._owners)<=8 and len(supervisor.events)<=512 and len(robot.tasks)<=256 and len(robot.requests)<=256
        await supervisor.cancel_all()
    asyncio.run(run())


def test_navigation_feedback_sequence_progress_destination_and_clock():
    async def run():
        nav=FakeNavigationBackend({'service_desk'})
        supervisor=BehaviorSupervisor(FakeRobotBackend(),nav)
        ack=await supervisor.submit(BehaviorRequest(intent='GUIDE_TO',destination_id='service_desk'))
        current=await supervisor.receive_feedback(nav.advance(ack.task_id,.5))
        for invalid in [current.model_copy(update={'sequence':0}),
                        current.model_copy(update={'sequence':2,'progress':.1}),
                        current.model_copy(update={'sequence':2,'destination_id':'invented'}),
                        current.model_copy(update={'sequence':2,'clock_domain':'ros_time'}),
                        current.model_copy(update={'sequence':2,'source_epoch':'restarted'}),
                        current.model_copy(update={'sequence':2,'turn_id':'stale-turn'})]:
            kept=await supervisor.receive_feedback(invalid)
            assert kept.status=='RUNNING' and kept.progress==.5
        await supervisor.cancel_all()
    asyncio.run(run())


def test_frame_restart_epoch_roi_and_foreign_production_clock_are_additive():
    frame=ColorFrame(np.zeros((8,8,3),np.uint8),0,'c920',8,8,10)
    buffer=Stage7FrameBuffer()
    epoch=buffer.source_epoch
    buffer.publish(frame,source_epoch=epoch,produced_at_s=20,produced_clock_domain='pi_monotonic')
    _,meta=encode_keyframe(buffer.latest(),now_s=10.1,max_age_s=1)
    assert meta['source_time_s']==10 and meta['time_semantics']=='host_read_complete'
    assert meta['produced_clock_domain']=='pi_monotonic' and meta['frame_contract_version']==1
    # Legacy same-frame callers remain compatible; remote/restarted adapters
    # must supply the additive epoch to get restart provenance validation.
    encode_keyframe(buffer.latest(),now_s=10.1,max_age_s=1,roi=FrameROI('c920',0,10,(0,0,4,4)))
    buffer.clear()
    with pytest.raises(ValueError,match='epoch_mismatch'): buffer.publish(frame,source_epoch=epoch)
    buffer.publish(frame)
    roi=FrameROI('c920',0,10,(0,0,4,4),epoch)
    with pytest.raises(ValueError,match='epoch_mismatch'): encode_keyframe(buffer.latest(),now_s=10.1,max_age_s=1,roi=roi)


def test_asr_provenance_rejects_duplicate_unmapped_clock_and_restart():
    async def run():
        async def stream(messages,info): yield '收到。'
        session=AgentSession(AgentRuntime(FunctionModel(stream_function=stream),BehaviorSupervisor(
            FakeRobotBackend(),FakeNavigationBackend(set())),AgentConfig()),clock=lambda:10)
        await session.receive('你好小柒')
        meta=ASRMetadata(source_id='realtek',source_epoch='first',sequence=1,received_at_s=10)
        await session.receive('你好',source='voice',asr_metadata=meta)
        await session.wait()
        for invalid in [meta,meta.model_copy(update={'clock_domain':'pi_monotonic'}),meta.model_copy(update={'source_epoch':'second'})]:
            await session.receive('你好',source='voice',asr_metadata=invalid)
            assert session.last_decision=='voice_rejected'
        assert len(session.runtime.outcomes)==1
        await session.close()
    asyncio.run(run())
