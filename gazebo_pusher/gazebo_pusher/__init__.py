from gym.envs.registration import register

register(
    id='Planar5DoF-v0',
    entry_point='gazebo_pusher.envs:GazeboPlanar5DofEnv',
)

