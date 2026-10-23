import tempfile
import unittest
from pathlib import Path

import yaml

from kart_mapping.bag_inputs import load_metadata


class BagMetadataTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.bag = Path(self.temp.name)
        self.path = self.bag / "metadata.yaml"
        self.metadata = {
            "version": 9,
            "storage_identifier": "mcap",
            "topics_with_message_count": [{
                "topic_metadata": {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage"},
                "message_count": 4,
            }],
        }

    def write(self):
        self.path.write_text(yaml.safe_dump({"rosbag2_bagfile_information": self.metadata}))

    def test_standard_metadata_is_returned_unchanged(self):
        self.write()
        before = self.path.read_bytes()
        self.assertEqual(load_metadata(self.bag), self.metadata)
        self.assertEqual(self.path.read_bytes(), before)

    def test_missing_root_reports_path_and_actual_keys(self):
        self.path.write_text("other_metadata: {}\n")
        with self.assertRaises(ValueError) as error:
            load_metadata(self.bag)
        for expected in (str(self.path.resolve()), "rosbag2_bagfile_information", "other_metadata", "job.json"):
            self.assertIn(expected, str(error.exception))

    def test_invalid_yaml_and_root_shapes(self):
        for text in ("", "scalar", "[]", "key: [", "rosbag2_bagfile_information: null"):
            with self.subTest(text=text):
                self.path.write_text(text)
                with self.assertRaisesRegex(ValueError, "metadata.yaml"):
                    load_metadata(self.bag)

    def test_missing_file(self):
        with self.assertRaisesRegex(ValueError, "metadata.yaml"):
            load_metadata(self.bag)

    def test_storage_and_topic_list_are_required(self):
        for key, value in (("storage_identifier", ""), ("storage_identifier", None), ("topics_with_message_count", {})):
            with self.subTest(key=key, value=value):
                original = self.metadata[key]
                self.metadata[key] = value
                self.write()
                with self.assertRaisesRegex(ValueError, key):
                    load_metadata(self.bag)
                self.metadata[key] = original

    def test_malformed_topic_entries(self):
        for entry in (None, {}, {"topic_metadata": {}}, {
            "topic_metadata": {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage"},
            "message_count": -1,
        }):
            with self.subTest(entry=entry):
                self.metadata["topics_with_message_count"] = [entry]
                self.write()
                with self.assertRaisesRegex(ValueError, r"topics_with_message_count\[0\]"):
                    load_metadata(self.bag)


if __name__ == "__main__":
    unittest.main()
