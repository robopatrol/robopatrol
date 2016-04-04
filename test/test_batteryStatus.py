import unittest
import batteryStatus

class CheckBattery(unittest.TestCase):

    def changingNetbookPower(self):

        batteryStatus = batteryStatus()
        isCharging = isCharging()

        # this test will fail until you change the Greeter to return this expected message
        self.assertEqual(batteryStatus(20), "Kobuki battery is low")
        self.assertEqual(batteryStatus(20), "Kobuki battery is fine")

        # this test will fail until you change the expected Value to 2
        #self.assertTrue(netbookPower(24), lowNetbookBattery)


        # isCharging
        self.assertEqual(isCharging(0), "Stopped charging")
        self.assertEqual(isCharging(1), "Pumping petrol at the station")

if __name__ == '__main__':
    unittest.main()