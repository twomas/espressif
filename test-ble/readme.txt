A demo for evaluating multiple ble connections

* Using Espressif branch release/v3.3 
* Install test-ble in one device (both advertiser and scanner) and monitor
* Install nRF Connect on several mobile phones (scanner) and connect to TBR_DEMO
* Install this application examples\bluetooth\gatt_server (advertiser) on several devices and monitor them (See names in demo.c for searched names, remote_device_name)

Adjust the test_task function according to your needs in demo.c in test-ble
This task toggles between advertising and scanning
