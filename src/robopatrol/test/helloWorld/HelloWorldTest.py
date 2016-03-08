import unittest
from HelloWorld import Greeter

class MyTestCase(unittest.TestCase):
    def test_default_greeting_set(self):
        greeter = Greeter()
        # this test will fail until you change the Greeter to return this expected message
        self.assertEqual(greeter.message, 'Hello world_')
        # this test will fail until you change the expected Value to 2
        self.assertEqual(greeter.addOne(self, 1), 3)

if __name__ == '__main__':
    unittest.main()