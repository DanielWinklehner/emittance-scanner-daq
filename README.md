# Emittance Scanner DAQ

Server and GUI for operating the MIST-1 emittance scanners

### TODO

 - Error handling
    - Code could get stuck if user tries to set a value that the device can't output
    - No defined action if user exits/stops communication during calibration


 - Bugs
    - Slow communication can cause messages to pile up (e.g. 'vset 10000poll') which will stop communication. Seems to only happen very rarely
    - Voltage regulator calibration not implemented (multi-point calibration preferred)
    - 1px offset in bottom row of scan histograms... No discernable negative effects other than it is annoying


 - User experience
    - Need to pick a way to save files when doing multiple scans
        1. Could add prefixes (e.g. file_v.csv, file_h.csv)
        2. Could append results of horizontal scan to vertical scan file
        3. Could add another column in the data to specify 'v' or 'h' with each data point
    - Save session properties
       1. Window: size, position, splitter dimensions
       2. Devices: calibration distances
       3. Scans: All fields could be saved
    - Fill in missing error message blocks
    - Fill in scan status labels & estimate time remaining
