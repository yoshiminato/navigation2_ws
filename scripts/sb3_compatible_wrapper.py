from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv
import numpy as np  

try:
    import supersuit as ss
    HAS_SUPERSUIT = True
except ImportError:
    HAS_SUPERSUIT = False
    print("Warning: supersuit not installed. Run: pip install supersuit")

class SB3CompatibleWrapper(VecEnv):
    """MarkovVectorEnvをSB3互換のVecEnvに変換するラッパー"""
    def __init__(self, markov_vec_env):
        self.markov_vec_env = markov_vec_env
        super().__init__(
            num_envs=markov_vec_env.num_envs,
            observation_space=markov_vec_env.observation_space,
            action_space=markov_vec_env.action_space
        )
    
    def reset(self):
        obs, info = self.markov_vec_env.reset()
        return obs
    
    def step_async(self, actions):
        self._actions = actions
    
    def step_wait(self):
        obs, rewards, dones, truncated, infos = self.markov_vec_env.step(self._actions)
        # SB3はdones1つのみを期待
        combined_dones = np.logical_or(dones, truncated)
        # Debug logging to help diagnose why VecMonitor may not record episodal scalars

        return obs, rewards, combined_dones, infos
    
    def close(self):
        self.markov_vec_env.close()
    
    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs
    
    def get_attr(self, attr_name, indices=None):
        return [getattr(self.markov_vec_env, attr_name)]
    
    def set_attr(self, attr_name, value, indices=None):
        setattr(self.markov_vec_env, attr_name, value)
    
    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        return [getattr(self.markov_vec_env, method_name)(*method_args, **method_kwargs)]
    
    def seed(self, seed=None):
        return [self.markov_vec_env.seed(seed)]

def wrap_env_for_sb3(env):
    """PettingZoo ParallelEnvをStable-Baselines3互換に変換"""
    if not HAS_SUPERSUIT:
        raise ImportError("supersuit is required. Install: pip install supersuit")
    
    env = ss.black_death_v3(env)  # エージェントの終了を処理
    
    # MarkovVectorEnvに変換
    env = ss.pettingzoo_env_to_vec_env_v1(env)
    
    # SB3互換ラッパーでラップ
    env = SB3CompatibleWrapper(env)
    
    return env