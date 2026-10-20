"""Nonblocking ROS parameter transactions, advanced by the HDMap executor."""
import time

from .parameter_policy import LABELS, catalog, changes_for, target
from .tuning import digest


class ParameterBridge:
    def __init__(self, bridge):
        from rcl_interfaces.srv import GetParameters, DescribeParameters, SetParametersAtomically
        self.bridge = bridge
        self.types = (GetParameters, DescribeParameters, SetParametersAtomically)
        self.clients = {}
        self.pending = None

    def clients_for(self, key):
        node, _ = target(key)
        if key not in self.clients:
            namespace = self.bridge.node.get_namespace().rstrip('/')
            self.clients[key] = tuple(self.bridge.node.create_client(kind, f'{namespace}/{node}/{suffix}')
                                      for kind, suffix in zip(self.types, ('get_parameters', 'describe_parameters', 'set_parameters_atomically')))
        return self.clients[key]

    def read(self, pending):
        _, names = target(pending['body']['target'])
        clients = self.clients_for(pending['body']['target'])
        requests = [kind.Request() for kind in self.types[:2]]
        for request in requests:
            request.names = list(names)
        pending['futures'] = [client.call_async(request) for client, request in zip(clients, requests)]

    def begin(self, action, body, reply, deadline):
        if self.pending is not None:
            reply.put({'error': 'パラメータ処理中です。結果確認後に再取得してください'})
            return
        try:
            if body.get('map_id') != self.bridge.identity:
                raise ValueError('NotebookとJetsonの地図が一致しません')
            key = body['target']
            target(key)
            if time.monotonic() >= deadline:
                raise ValueError('Request expired')
            if not all(c.service_is_ready() for c in self.clients_for(key)):
                if action == '/parameters_set':
                    raise ValueError('対象ノードのparameter serviceが起動していません')
                reply.put(dict(map_id=self.bridge.identity, target=key, available=False, fields=[],
                               reason='対象ノード未起動・接続待ち', revision=''))
                return
            if action == '/parameters_set' and self.bridge.issue():
                raise ValueError(self.bridge.issue())
            self.pending = dict(action=action, body=body, reply=reply, deadline=deadline, stage='read')
            self.read(self.pending)
        except Exception as error:
            self.pending = None
            reply.put({'error': str(error)})

    def snapshot(self, pending):
        key = pending['body']['target']
        _, names = target(key)
        values = pending['futures'][0].result().values
        descriptors = pending['futures'][1].result().descriptors
        if len(values) != len(names) or len(descriptors) != len(names):
            raise ValueError('Incomplete ROS parameter response')
        fields = []
        for name, value, descriptor in zip(names, values, descriptors):
            actual = {1: value.bool_value, 2: value.integer_value, 3: value.double_value,
                      4: value.string_value}.get(value.type)
            fields.append(dict(name=name, value=actual, type=value.type,
                               read_only=descriptor.read_only or value.type not in (2, 3, 4),
                               label=LABELS.get(name, name), description=descriptor.description,
                               constraints=descriptor.additional_constraints))
        result = catalog(key, fields)
        if key == 'map':
            node = self.bridge.node
            from .document import select
            lane = select(node.document, node.selected)
            result['line_choices'] = list(lane['lines']) if lane else []
            result['map_revision'] = self.bridge.revision()
            result['revision'] = digest({k: v for k, v in result.items() if k != 'revision'})
        return result

    def finish(self, result):
        pending = self.pending
        self.pending = None
        pending['reply'].put(result)

    def tick(self):
        pending = self.pending
        if pending is None:
            return
        try:
            if time.monotonic() >= pending['deadline']:
                dispatched = pending['stage'] in ('set', 'readback')
                raise ValueError('適用結果未確認です。値を再取得してください。自動再送は行いません' if dispatched else 'Parameter read timed out')
            if not all(f.done() for f in pending['futures']):
                return
            if pending['stage'] == 'set':
                response = pending['futures'][0].result()
                if not response.result.successful:
                    raise ValueError(response.result.reason)
                pending['stage'] = 'readback'
                self.read(pending)
                return
            snapshot = self.snapshot(pending)
            if pending['action'] == '/parameters_get' or pending['stage'] == 'readback':
                snapshot['map_id'] = self.bridge.identity
                if pending['stage'] == 'readback':
                    snapshot['applied'] = True
                    snapshot['readback_matches'] = all(next(row['value'] for row in snapshot['fields'] if row['name'] == name) == value
                                                      for name, value in pending['changes'].items())
                self.finish(snapshot)
                return
            issue = self.bridge.issue()
            if issue:
                raise ValueError(issue)
            changes = changes_for(snapshot, pending['body']['changes'], pending['body']['expected_revision'])
            if time.monotonic() + .3 >= pending['deadline']:
                raise ValueError('適用の応答待ち時間が不足しています。再取得してください')
            from rclpy.parameter import Parameter
            request = self.types[2].Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in changes.items()]
            pending['changes'] = changes
            pending['futures'] = [self.clients_for(pending['body']['target'])[2].call_async(request)]
            pending['stage'] = 'set'
        except Exception as error:
            self.finish({'error': str(error)})
