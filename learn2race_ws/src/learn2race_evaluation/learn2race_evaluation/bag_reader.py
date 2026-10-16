"""Minimal rosbag2 reader for offline analysis.

Reading the bag rather than subscribing live matters for reproducibility: the
same recording can be re-analysed after a change to the analysis, which is not
true of a measurement taken once from a live topic.
"""

# References (full entries: docs/references.md at the repository root):
#   [R16] rosbag2: rosbag2 reader API
# More details: README.md of this package

from typing import Dict, Iterator, List, Tuple

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:                                   # pragma: no cover
    rosbag2_py = None


class BagReader:

    def __init__(self, path: str, storage_id: str = 'sqlite3'):
        if rosbag2_py is None:
            raise RuntimeError(
                'rosbag2_py is unavailable. Source the ROS 2 environment first.')

        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
            rosbag2_py.ConverterOptions('', ''))

        self.types: Dict[str, str] = {
            t.name: t.type for t in self.reader.get_all_topics_and_types()}

    def topics(self) -> List[str]:
        return sorted(self.types)

    def messages(self, topic: str) -> Iterator[Tuple[int, object]]:
        """Yield (timestamp_ns, message) for one topic, in recorded order."""
        if topic not in self.types:
            raise KeyError(f'{topic} is not in this bag. Available: {self.topics()}')

        msg_type = get_message(self.types[topic])

        self.reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        self.reader.seek(0)

        while self.reader.has_next():
            name, data, stamp = self.reader.read_next()
            if name == topic:
                yield stamp, deserialize_message(data, msg_type)

    def timestamps(self, topic: str) -> List[float]:
        """Recording timestamps for a topic, in seconds."""
        return [stamp * 1e-9 for stamp, _ in self.messages(topic)]
