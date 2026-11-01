import json
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import pytest
from recording_remote import request, select_service, STATUS_TYPE


def test_discovery_requires_unique_and_same_manager():
    assert select_service([]) is None
    assert select_service([('/robot/bag/recording_status', [STATUS_TYPE])]) == '/robot/bag/recording_status'
    with pytest.raises(ValueError):
        select_service([(x + '/bag/recording_status', [STATUS_TYPE]) for x in ('/a', '/b')])
    with pytest.raises(ValueError):
        select_service([('/bag/recording_status', [STATUS_TYPE])], '/old/bag/recording_status')


@pytest.mark.parametrize('action,enabled', [('recording_start', True), ('recording_stop', False), ('recording_status', None)])
@pytest.mark.parametrize('timeout', [False, True])
def test_single_request_and_cleanup(action, enabled, timeout):
    future = Mock()
    future.done.return_value = not timeout
    future.result.return_value = NS(success=True, message=json.dumps({'schema': 'kart.recording.v1', 'phase': 'stopping'}))
    client = Mock()
    client.call_async.return_value = future
    node = Mock()
    node.get_service_names_and_types.return_value = [('/bag/recording_status', [STATUS_TYPE])]
    node.create_client.return_value = client
    ros = Mock()
    ros.create_node.return_value = node
    # The first discovery poll must spin, using the initialized context.
    node.get_service_names_and_types.side_effect = [[], [('/bag/recording_status', [STATUS_TYPE])]]
    executor = ros.executor_factory.return_value
    def same_context(*args, **kwargs):
        assert kwargs['executor'] is executor
    ros.spin_once.side_effect = same_context
    ros.spin_until_future_complete.side_effect = same_context
    kind = NS(Request=lambda: NS())
    with patch.dict(sys.modules, {'rclpy': ros, 'rclpy.executors': NS(SingleThreadedExecutor=ros.executor_factory), 'std_srvs': NS(), 'std_srvs.srv': NS(Trigger=kind, SetBool=kind)}):
        if timeout:
            with pytest.raises(ValueError, match='自動再送せず'):
                request(action, '/bag/recording_status')
            future.cancel.assert_called_once()
        else:
            assert request(action, '/bag/recording_status')['phase'] == 'stopping'
    client.call_async.assert_called_once()
    if enabled is not None:
        assert client.call_async.call_args.args[0].data is enabled
        assert node.create_client.call_args.args[1] == '/bag/set_recording'
    ros.executor_factory.assert_called_once_with(context=ros.Context.return_value)
    ros.spin_once.assert_called_once()
    ros.spin_until_future_complete.assert_called_once()
    executor.shutdown.assert_called_once()
    node.destroy_node.assert_called_once()
    ros.shutdown.assert_called_once()


def test_mutation_requires_previous_status():
    with pytest.raises(ValueError):
        request('recording_start')
