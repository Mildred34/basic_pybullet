#!/usr/bin/env python3
import generator
import time
import gym
import logging,os,sys
sys.path.append(os.path.dirname(__file__)+"/logs")
from logs import _log_handling as log
from functions import DEBUG_LEVEL, LOG_PATH

def init():
    global logger
    path_log = os.path.dirname(__file__) + LOG_PATH()

    logging.info("Début du script")

    #Initialisation du mode debug ou release
    # Choix du mode DEBUG ou RELEASE
    if(DEBUG_LEVEL() == log.DEBUG_LEVEL.DEBUG_SOFT):
        log.config["filename_debug"] = path_log + "debug"
        logger = log.factory.create('DEBUG',**log.config)
        logger.info("Choix du mode: DEBUG")
    else:
        log.config["filename_release"] = path_log + "release"
        logger = log.factory.create('RELEASE',**log.config)
        logger.info("Choix du mode: RELEASE")


def main():
    init()
    env = gym.make('GraspingGenerator-v0')
    ob = env.reset()

    while True:
        ob, _, done, _ = env.step(action = None)
        #env.render()

        if done:
            ob = env.reset()
            time.sleep(1/30)
            logger.info("Fin du script")


if __name__ == '__main__':
    main()