import errno
import json
import os
import subprocess
import shutil

CROSSBAR_PATH = os.path.join(os.path.join(os.path.expanduser('~'), 'tmp'), '.crossbar')

CROSSBAR_CONFIG_JSON = {
    "version": 2,
    "controller": {},
    "workers": [
        {
            "type": "router",
            "options": {
                "pythonpath": [
                    ".."
                ]
            },
            "realms": [
                {
                    "name": "ros",
                    "roles": [
                        {
                            "name": "anonymous",
                            "permissions": [
                                {
                                    "uri": "ros.",
                                    "match": "prefix",
                                    "allow": {
                                        "call": True,
                                        "register": True,
                                        "publish": True,
                                        "subscribe": True
                                    },
                                    "disclose": {
                                        "caller": False,
                                        "publisher": False
                                    },
                                    "cache": True
                                }
                            ]
                        }
                    ]
                }
            ],
            "transports": [
                {
                    "type": "web",
                    "endpoint": {
                        "type": "tcp",
                        "port": 11111
                    },
                    "paths": {
                        "ws": {
                            "type": "websocket"
                        },
                        "lp": {
                            "type": "longpoll"
                        }
                    }
                }
            ]
        }
    ]
}


def crossbar_create_config(port: int) -> None:
    global CROSSBAR_PATH
    CROSSBAR_PATH = os.path.join(os.path.join(os.path.expanduser('~'), os.path.join('tmp', 'crossbar_%d' % port)), '.crossbar')
    try:
        os.makedirs(CROSSBAR_PATH)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
    with open(os.path.join(CROSSBAR_PATH, 'config.json'), "w") as file:
        CROSSBAR_CONFIG_JSON["workers"][0]["transports"][0]["endpoint"]["port"] = port
        file.write(json.dumps(CROSSBAR_CONFIG_JSON, ensure_ascii=False))

def crossbar_start_server(port: int) -> str:
    crossbar_create_config(port)
    if shutil.which('crossbar') is None:
        try:
            import rospy
            rospy.logerr("shutil.which('crossbar'): Could not find [crossbar], please check your PATH variable.")
        except:
            import sys
            sys.stderr.write("shutil.which('crossbar'): Could not find [crossbar], please check your PATH variable.")
        return ""

    print(shutil.which('screen'), "-dmS", "_crossbar_server_%d" % port, shutil.which('crossbar'), "start", "--cbdir", CROSSBAR_PATH)
    p = subprocess.Popen([shutil.which('screen'), "-dmS", "_crossbar_server_%d" % port, shutil.which('crossbar'), "start", "--cbdir", CROSSBAR_PATH])
    return CROSSBAR_PATH
