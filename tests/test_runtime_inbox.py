"""Ingress capacity and local-control priority are independent of asyncio speed."""
from queue import Empty
from concurrent.futures import ThreadPoolExecutor

import pytest
from companionbot_behavior.runtime.inbox import SessionInbox


def test_full_inbox_rejects_without_losing_prior_admitted_input():
    inbox = SessionInbox(lambda item: item == "/stop", capacity=2)
    assert inbox.put("first") and inbox.put("second")
    assert not inbox.put("overflow")
    assert inbox.get_nowait() == "first"
    assert inbox.get_nowait() == "second"
    with pytest.raises(Empty):
        inbox.get_nowait()
    assert inbox.snapshot()["rejected"] == 1


def test_stop_supersedes_backlog_and_survives_a_following_burst():
    inbox = SessionInbox(lambda item: item == "/stop", capacity=2)
    inbox.put("old intent")
    inbox.put("old intent 2")
    assert inbox.put("/stop")
    assert inbox.put("later intent")
    assert not inbox.put("overflow")
    assert inbox.get_nowait() == "/stop"
    assert inbox.snapshot()["superseded"] == 2
    inbox.close()
    assert not inbox.put("/stop")
    assert inbox.snapshot()["pending"] == 0


def test_concurrent_ingress_has_a_hard_memory_bound_and_exact_admission_count():
    inbox = SessionInbox(lambda _: False)
    with ThreadPoolExecutor(max_workers=4) as executor:
        admitted = sum(executor.map(inbox.put, range(1024)))
    assert admitted == inbox.snapshot()["accepted"] == inbox.snapshot()["pending"] == 64
    assert inbox.snapshot()["rejected"] == 960
