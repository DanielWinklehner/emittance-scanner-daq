# Emittance Scanner DAQ

Server and GUI for operating the MIST-1 emittance scanners

### TODO

 - Error handling
    - (Done: User cannot enter values exceeding stepper limits) ~~Code could get stuck if user tries to set a value that the device can't output~~
    - No defined action if user exits/stops communication during calibration


 - Bugs
    - Slow communication can cause messages to pile up (e.g. 'vset 10000poll') which will stop communication. Seems to only happen very rarely
    - (Done) ~~Voltage regulator calibration not implemented (multi-point calibration preferred)~~
    - 1px offset in bottom row of scan histograms... No discernable negative effects other than it is annoying


 - User experience
    - (Done) ~~Need to pick a way to save files when doing multiple scans~~
        1. ~~Could add file endings (e.g. file_v.csv, file_h.csv)~~ (Winner)
        2. ~~Could append results of horizontal scan to vertical scan file~~
        3. ~~Could add another column in the data to specify 'v' or 'h' with each data point~~
    - (Done) ~~Save session properties~~
       1. ~~Window: size, position, splitter dimensions~~
       2. ~~Devices: calibration distances~~
       3. ~~Scans: All fields could be saved~~
    - (Done) ~~Fill in missing error message blocks~~
    - Fill in scan status labels & estimate time remaining
    - Automatically repeat calibration at user-specified intervals
