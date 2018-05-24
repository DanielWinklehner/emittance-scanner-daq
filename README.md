# Emittance Scanner DAQ

Server and GUI for operating the MIST-1 emittance scanners

### TODO

 - Data output
    - Might want to save voltage regulator std. dev of samples.
    - User-adjustable number of voltage regulator and pico sampling


 - Error handling
    - (Done: User cannot enter values exceeding stepper limits and vreg limits are now user-adjustable) ~~Code could get stuck if user tries to set a value that the device can't output~~
    - No defined action if user exits/stops communication during calibration


 - Bugs
    - Slow communication can cause messages to pile up (e.g. 'vset 10000poll') which will stop communication. This is hard to reproduce and only happens rarely.
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
    - (Done) ~~Fill in scan status labels & estimate time remaining~~
    - Automatically repeat calibration at user-specified intervals
    - Save previous scans to the review tab
    - (Done) ~~Save an image of the scan when the scan completes~~
    - (Done) ~~User-editable metadata fields~~


 - Code
    - I would like to refactor portions of the code to make the above todo list items easier to implement.
        1. Scans as their own object so multiple scans can be saved and accessed within the GUI.
        2. The calibrator and daq objects follow the same design principle: Make a QObject, move it to a thread, run the object's method to completion, then safely delete the object and thread before allowing the user to create another instance. This could be encapsulated somehow.
        3. A bit hard to add features to the 2D histogram, like axis ticks and labels. Ideally refactor this code to its own class.
    - Devices are currently dictionaries. They all only use the global calibrate function, and this works well. However, they may need their own functions at some point, so maybe it would be wise to refactor the device dictionaries into classes.
