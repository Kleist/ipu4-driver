import time

class ManualScope(object):
    """
    ManualScope implements parts of the interface of ambu_test_library.scope, which allows
    a developer to run some automated scope tests locally.
    """

    def __init__(self, target):
        self._target = target

    def disable_all(self):
        while (
                self._target.scope.is_connected(scope_idx=0) or 
                self._target.scope.is_connected(scope_idx=1) or 
                self._target.scope.is_connected(scope_idx=2)
                ):
            print("ManualScope: Please remove all scopes (test continues when scopes are no longer detected)")
            time.sleep(1)

    def enable(self, scope):
        while not (self._target.scope.is_ready(scope_idx=0, scope_id=scope.id)
                or self._target.scope.is_ready(scope_idx=1, scope_id=scope.id)
                or self._target.scope.is_ready(scope_idx=2, scope_id=scope.id)):
            print(f"ManualScope: Please insert scope with ID 0x{scope.id:X} (test continues when scope is detected)")
            time.sleep(1)

    def disable(self, scope):
        while True:
            scopes_inserted = self._target.cmd("cat /sys/class/scope/scope*/type || true").strip().split()
            if not f"{scope.id}" in scopes_inserted:
                break
            scopes_inserted_string = ", ".join(f"0x{s:X}" for s in scopes_inserted)
            print(f"ManualScope: Please remove scope with ID 0x{scope.id:X} (detected {scopes_inserted_string}, test continues when scope is no longer detected)")
            time.sleep(1)
