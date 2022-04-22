import pytest
from mock.mock import Mock

from lisdf.plan_executor.executor import CommandExecutor


class _ConcreteExecutor(CommandExecutor):
    @property
    def duration(self) -> float:
        return 1.0

    def execute(self, current_time: float) -> None:
        pass


@pytest.fixture
def executor() -> _ConcreteExecutor:
    return _ConcreteExecutor(Mock(), Mock(), 0.0)


def test_command_executor_end_time(executor):
    assert executor.end_time == 1.0


def test_command_executor_finished(executor):
    assert not executor.finished(0.0)
    assert not executor.finished(0.5)
    assert executor.finished(1.0)
