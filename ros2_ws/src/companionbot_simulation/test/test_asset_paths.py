"""Relocation keeps the original plant/config paths and encoder type identity."""
from companionbot_contracts.paths import repository_root
from companionbot_control.estimation.longitudinal_estimation import DEFAULT_ESTIMATOR_CONFIG_PATH
from companionbot_control.sensors.encoder import QuadratureEncoderProfile
from companionbot_simulation.core import minisegway, virtual_imu


def test_original_assets_and_encoder_contract_are_shared():
    root = repository_root() / "models/minisegway"
    assert minisegway.DEFAULT_MODEL_PATH == root / "mini_segway.xml"
    assert minisegway.DEFAULT_ENCODER_PROFILE_PATH == root / "encoder_profile.json"
    assert DEFAULT_ESTIMATOR_CONFIG_PATH == root / "longitudinal_estimator_config.json"
    assert virtual_imu.DEFAULT_IMU_HARDWARE_CONFIG_PATH == root / "imu_hardware_config.json"
    assert minisegway.QuadratureEncoderProfile is QuadratureEncoderProfile
    assert all(path.is_file() for path in [minisegway.DEFAULT_MODEL_PATH,
                minisegway.DEFAULT_ENCODER_PROFILE_PATH, DEFAULT_ESTIMATOR_CONFIG_PATH,
                virtual_imu.DEFAULT_IMU_HARDWARE_CONFIG_PATH])
