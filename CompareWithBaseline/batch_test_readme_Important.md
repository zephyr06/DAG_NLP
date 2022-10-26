## Must do things before any test:
### General settings:
* Update `PROJECT_PATH` in `sources/Utils/Parameters.h` to correct path
* Update `ROOT_PATH` in `CompareWithBaseline/RTDA2CoresPerformance/CompareRTDA2CoresPerformance.sh` for RTDA test
* Update `ROOT_PATH` in `CompareWithBaseline/SensorFusion2CoresPerformance/CompareSensorFusion2CoresPerformance.sh` for Sensor Fusion test
* Update `ROOT_PATH` in `CompareWithBaseline/build_tar_ball_for_backup.sh` to back up tar balls

### runVerucchiOnlyRTAS2023 settings
* Update `ROOT_PATH` in `CompareWithBaseline/VerucchiOnlyRTAS2023/runVerucchiOnlyRTAS2023.sh` for verucchi only tests
* Update `TaskNumberArray` in `CompareWithBaseline/VerucchiOnlyRTAS2023/runVerucchiOnlyRTAS2023.sh` for verucchi only tests
* Make sure you are using RTDATasksets in `TASKSETS_PATH` of `runVerucchiOnlyRTAS2023.sh`
