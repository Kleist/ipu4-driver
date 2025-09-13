import time
import ambu_test_library as atl

def get_scope_by_id(scope_id):
    scopes = atl.scope.get_by_id(scope_id)
    assert len(scopes) == 1
    return scopes[0]

def insert_scope(test_agent, scope_id):
    scope = get_scope_by_id(scope_id)
    print(f"Inserting scope with ID 0x{scope_id:x}")
    test_agent.scope.enable(scope)

def remove_all_scopes(test_agent):
    print("Removing all scopes")
    test_agent.scope.disable_all()

def remove_scope(test_agent, scope_id):
    scope = get_scope_by_id(scope_id)
    print(f"Removing scope with ID 0x{scope_id:x}")
    test_agent.scope.disable(scope)
