from gym.envs.registration import register
register(
    id='GraspingGenerator-v0',
    entry_point='generator.envs:GraspingGeneratorEnv'
)

register(
    id='GraspingGenerator-mini',
    entry_point='generator.envs:GraspingGeneratorSimpleenv'
)