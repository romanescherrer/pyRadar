from . import gtrack_module
from . import gtrack_test
from . import ekf_utils
import numpy as np


class EKF:
    """Extended Kalman Filter with built in tracking

    Attributes:
        point_cloud (ndarray): Array of point objects
        target_desc (ndarray): Array of detected objects
        t_num (int): Number of detected objects
        h_track_module (object): Tracking meta-data
        num_points (int): Number of detected points

    """

    def __init__(
        self,
        deltaT=0.05,
        maxNumPoints=250,
        maxNumTracks=20,
        stateVectorType=ekf_utils.gtrack_STATE_VECTOR_TYPE().gtrack_STATE_VECTORS_2DA,
        initialRadialVelocity=0,
        maxRadialVelocity=20,
        radialVelocityResolution=0,
        maxAcceleration=2,
        verbose=ekf_utils.gtrack_VERBOSE_TYPE().gtrack_VERBOSE_NONE,
        app_scenery_params=ekf_utils.gtrack_sceneryParams(
            numBoundaryBoxes=0,
            numStaticBoxes=0,
            bound_box=[(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)],
            static_box=[(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)],
        ),
        app_gating_params=ekf_utils.gtrack_gatingParams(
            volume=4.0, params=[(3.0, 2.0, 0.0)]
        ),
        app_allocation_params=ekf_utils.gtrack_allocationParams(
            snrThre=50.0,
            velocityThre=0.01,
            pointsThre=20,
            maxDistanceThre=0.75,
            maxVelThre=2,
        ),
        app_std_sigma=ekf_utils.gtrack_varParams(
            lengthStd=0.289, widthStd=0.289, dopplerStd=1.0
        ),
    ):
        self.point_cloud = np.array(
            [
                ekf_utils.gtrack_measurementPoint()
                for _ in range(ekf_utils.MAXNUMBERMEASUREMENTS)
            ]
        )

        self.target_desc = np.array(
            [ekf_utils.gtrack_targetDesc() for _ in range(ekf_utils.MAXNUMBERTRACKERS)]
        )

        # customized configuration params
        app_scenery_params = app_scenery_params
        app_gating_params = app_gating_params
        app_allocation_params = app_allocation_params
        app_std_sigma = app_std_sigma

        self.t_num = np.zeros(1)

        config = ekf_utils.gtrack_moduleConfig()
        adv_params = ekf_utils.gtrack_advancedParameters()

        config.stateVectorType = stateVectorType
        config.verbose = verbose

        config.deltaT = deltaT
        config.maxRadialVelocity = maxRadialVelocity
        config.radialVelocityResolution = radialVelocityResolution
        config.maxAcceleration = maxAcceleration
        config.maxNumPoints = maxNumPoints
        config.maxNumTracks = maxNumTracks
        config.initialRadialVelocity = initialRadialVelocity

        adv_params.gatingParams = app_gating_params
        adv_params.sceneryParams = app_scenery_params
        adv_params.allocationParams = app_allocation_params
        adv_params.variationParams = app_std_sigma

        config.advParams = adv_params

        self.h_track_module = gtrack_test.create(config)
        self.num_points = None

    def update_point_cloud(self, ranges, azimuths, dopplers, snrs):
        """Update step of the EKF

        Args:
            ranges: Ordered ranges of the detected points
            azimuths: Ordered azimuths of the detected points
            dopplers: Ordered dopplers of the detected points
            snrs: Ordered snrs of the detected points

        Returns:
            None
        """
        for distance, azimuth, doppler, snr, idx in zip(
            ranges, azimuths, dopplers, snrs, range(len(ranges))
        ):
            self.point_cloud[idx].range = distance
            self.point_cloud[idx].angle = azimuth
            self.point_cloud[idx].doppler = doppler
            self.point_cloud[idx].snr = snr
        self.num_points = len(ranges)

    def step(self):
        """Step the EKF

        Returns:
            Tuple [list, int]:
                1. list: Information about detected objects
                #. int: Number of detected objects

        """
        gtrack_module.step(
            self.h_track_module,
            self.point_cloud,
            None,
            self.num_points,
            self.target_desc,
            self.t_num,
            0,
        )

        return self.target_desc, self.t_num
