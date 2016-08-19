* Tracking algorithms for pyranometer mount	*
* Michael Lipski				*
* Summer 2016					*

Repository contains sketches used to control the pyranometer mount consisting of two Zaber T-RS60A rotational stages.  Tracking methods include open-loop (feed forward), closed-loop (feedback), and dual-mode (both).  Also included are sketches used in testing various Zaber stage commands in binary protocol.

Copy folders from "libraries" into your "Arduino/libraries" folder and restart the IDE before using.

* *  Sketch Descriptions  * *
external_pyranometer_tracker: Pyranometer tracking using external CdS photocell-based tracker
external_pyranometer_tracker_lcd: Pyranometer tracking using external CdS photocell-based tracker; LCD output
photoresistor_detector_read_test: Reads the voltages from the photocell voltage dividers and prints values
photoresistor_detector_read_test_lcd: Photocell voltage reader with LCD screen output
pyranometer_tracker: Dual-mode tracking for DNI pyranometer
pyranometer_tracker_feedback: Closed-loop tracking for DNI pyranometer
pyranometer_tracker_feedback_test: Active test sketch for closed-loop pyranometer tracking
pyranometer_tracker_feedforward: Open-loop tracking for DNI pyranometer
pyranometer_tracker_test_indoor: Moves rotational stages to user-input values for azimuth and zenith 
