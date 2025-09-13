import pytest

import actions

def test_check_modules_loaded(target):
    modules = actions.loaded_modules(target)
    assert "ambu_monitor" in modules
    if target.type == 'abox2':
        assert "amcu" in modules
    else:
        assert "b503ec" in modules
