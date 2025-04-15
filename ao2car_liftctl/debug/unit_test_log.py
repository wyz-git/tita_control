import unittest
from src.tools.log import Log


class TestLog(unittest.TestCase):
    def setUp(self):
        self.Log = Log("unit_test_log")

    def test_01_run(self):
        self.Log.debug("this is debug")
        print("===============")

    def test_02_run(self):
        self.Log.error("this is error")
        print("===============")

    def test_03_run(self):
        self.Log.info("this is info")
        print("===============")

    def test_04_run(self):
        self.Log.output("this is output")
        print("===============")


if __name__ == "__main__":
    unittest.main()
