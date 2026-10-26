"""CPU policy synchronization and shutdown only; no simulated model scores."""
import os
import threading

import pytest

from companionbot_perception.runtime.rendered import (
    _InferenceThreadBudget, RenderedPerceptionRuntime,
)


class PublicThreadAPI:
    def __init__(self):
        self.local = threading.local()
        self.calls = []
        self.lock = threading.Lock()

    def get_num_threads(self):
        return getattr(self.local, 'threads', 7)

    def set_num_threads(self, threads):
        self.local.threads = threads
        with self.lock:
            self.calls.append((threading.get_native_id(), threads))


@pytest.mark.parametrize('threads', [0, -1, True, 1.5, '2'])
def test_invalid_budget_rejected_before_loading_models_or_camera(threads):
    with pytest.raises(ValueError, match='invalid_inference_threads'):
        RenderedPerceptionRuntime(camera_path='unopened', protocol_path='unopened',
            source_epoch='epoch', output_dir='unused', on_update=lambda update: None,
            character_asset_lock_sha256='unit', inference_threads=threads)


def test_budget_cannot_exceed_available_cpu_set():
    count = len(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else os.cpu_count() or 1
    with pytest.raises(ValueError, match='available_cpus'):
        _InferenceThreadBudget(count+1)


def test_default_keeps_original_library_policy_without_barrier_or_setter():
    api = PublicThreadAPI()
    policy = _InferenceThreadBudget(None, backend=api)
    assert policy.actual() is None
    assert policy.check() is None
    assert policy.after_first_inference('depth', threading.Event()) is True
    policy.abort()
    assert api.calls == []
    assert policy.report()['requested_threads'] is None


def test_both_lazy_model_setups_finish_before_public_setters_and_next_inference():
    api = PublicThreadAPI()
    policy = _InferenceThreadBudget(1, backend=api, timeout_s=1.)
    stop = threading.Event()
    first_started, second_setup = threading.Event(), threading.Event()
    outcomes, errors = [], []

    def worker(name):
        try:
            if name == 'tracking':
                first_started.set()
            else:
                second_setup.set()
            assert policy.after_first_inference(name, stop)
            assert second_setup.is_set()
            # Reaching another compute call requires both setters to finish.
            assert len(api.calls) == 2
            outcomes.append(policy.check())
        except BaseException as error:
            errors.append(error)

    first = threading.Thread(target=worker, args=('tracking',))
    second = threading.Thread(target=worker, args=('depth',))
    first.start()
    assert first_started.wait(1.)
    assert api.calls == []
    second.start()
    for thread in (first, second):
        thread.join(2.)
        assert not thread.is_alive()
    assert not errors
    assert outcomes == [1, 1]
    assert {row['after_original_first_predict_threads'] for row in policy.channels.values()} == {7}
    assert {row['after_public_setter_threads'] for row in policy.channels.values()} == {1}


def test_missing_worker_fails_with_bounded_setup_wait():
    policy = _InferenceThreadBudget(1, backend=PublicThreadAPI(), timeout_s=.02)
    with pytest.raises(RuntimeError, match='setup_incomplete'):
        policy.after_first_inference('tracking', threading.Event())


def test_failure_closes_slots_and_releases_other_worker_from_setup_barrier():
    from companionbot_perception.core.detection_stream import LatestFrameSlot
    runtime = RenderedPerceptionRuntime.__new__(RenderedPerceptionRuntime)
    runtime.inference_thread_budget = _InferenceThreadBudget(1, backend=PublicThreadAPI())
    runtime._state_lock = threading.Lock()
    runtime.errors, runtime.stop = [], threading.Event()
    runtime.slots = {'tracking': LatestFrameSlot(), 'depth': LatestFrameSlot()}
    started, outcome = threading.Event(), []

    def waiting_worker():
        started.set()
        outcome.append(runtime.inference_thread_budget.after_first_inference('tracking', runtime.stop))

    worker = threading.Thread(target=waiting_worker)
    worker.start()
    assert started.wait(1.)
    runtime._fail(RuntimeError('original_depth_worker_failed'))
    worker.join(1.)
    assert not worker.is_alive()
    assert outcome == [False]
    assert runtime.stop.is_set()
    assert all(slot.closed for slot in runtime.slots.values())
    assert runtime.errors == ["RuntimeError('original_depth_worker_failed')"]


def test_budget_drift_is_an_error_instead_of_silent_performance_fallback():
    api = PublicThreadAPI()
    policy = _InferenceThreadBudget(1, backend=api)
    api.set_num_threads(1)
    assert policy.check() == 1
    api.set_num_threads(7)
    with pytest.raises(RuntimeError, match='not_honored'):
        policy.check()
