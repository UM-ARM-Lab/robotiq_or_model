#!/usr/bin/env python
import IPython
import openravepy


if __name__ == "__main__":
    env = openravepy.Environment()
    env.Load('obj_env.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    i = 0  
    
    IPython.embed()