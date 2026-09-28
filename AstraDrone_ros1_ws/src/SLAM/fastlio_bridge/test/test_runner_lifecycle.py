"""Lifecycle regression without starting ROS/PX4 or touching existing nodes."""
import importlib.util
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/run_dynamic_comparison.py'
SPEC = importlib.util.spec_from_file_location('comparison_runner', SCRIPT)
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


class RunnerLifecycleTest(unittest.TestCase):
    def test_native_writer_waited_before_read(self):
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / 'test.bag'
            process = Mock(returncode=0)
            process.poll.return_value = None
            # The completed file appears only after the writer finishes.
            process.wait.side_effect = lambda **kw: bag.touch()
            with patch.object(runner.rosbag, 'Bag') as reader:
                reader.return_value.__enter__.return_value.get_message_count.return_value = 1
                runner.finish_recording(process, bag)
                process.send_signal.assert_called_once_with(signal.SIGINT)
                process.wait.assert_called_once_with(timeout=25)
                reader.assert_called_once_with(str(bag), 'r')

    def test_incomplete_and_empty_bags_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / 'test.bag'
            process = Mock(returncode=0)
            process.poll.return_value = 0
            with self.assertRaises(RuntimeError):
                runner.finish_recording(process, bag)
            bag.touch()
            active = Path(str(bag) + '.active')
            active.touch()
            with self.assertRaises(RuntimeError):
                runner.finish_recording(process, bag)
            active.unlink()
            with patch.object(runner.rosbag, 'Bag') as reader:
                reader.return_value.__enter__.return_value.get_message_count.return_value = 0
                with self.assertRaises(RuntimeError):
                    runner.finish_recording(process, bag)

    def test_repeated_sigint_during_child_wait(self):
        # Real waiting child, signals only to this test process. Mock orphan
        # discovery so this regression cannot stop another ROS session.
        with tempfile.TemporaryDirectory() as folder:
            run = runner.CaseRunner(Path(folder), Path(folder)/'case', os.environ.copy())
            child = run.start('dummy', [sys.executable, '-c',
                'import signal,time; signal.signal(signal.SIGINT, lambda s,f: time.sleep(.5)); time.sleep(.5)'])
            time.sleep(.15)
            previous = signal.signal(signal.SIGINT, runner.cleanup_interrupt)
            repeat = threading.Timer(.15, lambda: os.kill(os.getpid(), signal.SIGINT))
            repeat.start()
            try:
                with patch.object(runner.subprocess, 'run', return_value=Mock(stdout='')):
                    run.stop()
                self.assertIsNotNone(child.poll())
                self.assertTrue(all(stream.closed for stream in run.logs))
            finally:
                repeat.join()
                signal.signal(signal.SIGINT, previous)
                if child.poll() is None:
                    child.kill()
                    child.wait()


if __name__ == '__main__':
    unittest.main()
