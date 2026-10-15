import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import onnx
from onnx import TensorProto, helper

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('base_runner', REPO / 'tools/test_sfyg_base_policy.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def zero_network(path, nin, nout):
    tensor = helper.make_tensor('zero', TensorProto.FLOAT, [nout], [0.] * nout)
    graph = helper.make_graph([helper.make_node('Constant', [], ['output'], value=tensor)],
        'zero', [helper.make_tensor_value_info('input', TensorProto.FLOAT, [nin])],
        [helper.make_tensor_value_info('output', TensorProto.FLOAT, [nout])])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)])
    model.ir_version = 8
    onnx.save(model, str(path))


class BaseRunnerTest(unittest.TestCase):
    def test_base_input_order_and_invalid_width(self):
        value = module.compose_base_input(np.ones(3), np.ones(42) * 2, np.ones(3) * 3)
        np.testing.assert_array_equal(value, [1.] * 3 + [2.] * 42 + [3.] * 3)
        with self.assertRaises(ValueError):
            module.compose_base_input(np.zeros(4), np.zeros(42), np.zeros(3))

    def test_mujoco_short_smoke_and_reject_wholebody_network(self):
        if not (REPO.parent / 'tron2_rl_deploy_python/controllers/sfyg_contract.py').exists():
            self.skipTest('requires sibling deploy project')
        with tempfile.TemporaryDirectory(prefix='base-smoke-', dir=REPO / 'tests') as temp:
            root = Path(temp)
            zero_network(root / 'encoder.onnx', 420, 3)
            zero_network(root / 'policy.onnx', 48, 10)
            with patch('sys.argv', ['base-test', '--model-dir', str(root), '--duration', '.06']):
                module.main()
            zero_network(root / 'policy.onnx', 78, 10)
            with self.assertRaises(ValueError):
                module.session(root / 'policy.onnx', 48, 10)


if __name__ == '__main__':
    unittest.main()
