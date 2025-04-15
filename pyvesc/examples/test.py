import pyvesc.VESC.messages.setters
import pyvesc.VESC.messages.getters

print("--- Available Getters ---")
# GetMcConfRequest 가 있는지 확인
print([name for name in dir(pyvesc.messages.getters) if not name.startswith("_")])

print("\n--- Available Setters ---")
# 설정(Set) 및 저장(Store/Write)과 관련된 이름을 찾아보세요.
# 예: SetMCConf, SetConfiguration, WriteMCConf, StoreConfiguration 등
print([name for name in dir(pyvesc.messages.setters) if not name.startswith("_")])
