nrfutil.exe settings generate --family NRF52 --application nrf52832_xxaa_app.hex --application-version 3 --bootloader-version 2 --bl-settings-version 2 bootloader_settings.hex
mergehex.exe --merge nrf52832_xxaa_bootloader.hex s132_nrf52_7.0.1.hex --output production_final1.hex 
mergehex.exe --merge production_final1.hex nrf52832_xxaa_app.hex --output production_final2.hex
mergehex.exe --merge production_final2.hex bootloader_settings.hex --output production_final.hex
del bootloader_settings.hex
del production_final1.hex
del production_final2.hex