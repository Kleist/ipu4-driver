import time

import manual_scope

class ManualAgent(object):
    """
    Allow some tests that uses the test_agent to run. Actions usually done
    by the test_agent are instead printed to the user.
    """
    def __init__(self, target):
        self.scope = manual_scope.ManualScope(target)
