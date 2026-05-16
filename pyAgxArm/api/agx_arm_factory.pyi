# agx_arm_factory.pyi
from typing_extensions import Literal
from typing import Any, TypeVar, overload
from ..protocols.can_protocol.drivers.piper import PiperDriverDefault
from ..protocols.can_protocol.drivers.nero import NeroDriverDefault
from ..protocols.can_protocol.drivers.piper_h import PiperHDriverDefault
from ..protocols.can_protocol.drivers.piper_l import PiperLDriverDefault
from ..protocols.can_protocol.drivers.piper_x import PiperXDriverDefault

class NeroCanDefaultConfig():
    pass
class PiperCanDefaultConfig():
    pass
class PiperHCanDefaultConfig():
    pass
class PiperLCanDefaultConfig():
    pass
class PiperXCanDefaultConfig():
    pass

@overload
def create_agx_arm_config(
    robot: Literal["nero"],
    comm: Literal["can"],
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> NeroCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper"],
    comm: Literal["can"],
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_h"],
    comm: Literal["can"],
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperHCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_l"],
    comm: Literal["can"],
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperLCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_x"],
    comm: Literal["can"],
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperXCanDefaultConfig: ...

T = TypeVar("T", bound=Any)

class AgxArmFactory:

    @classmethod
    @overload
    def create_arm(cls, config: None, **kwargs) -> None:
        """
        Create a robotic arm Driver instance.
        """
        ...

    @classmethod
    @overload
    def create_arm(cls, config: NeroCanDefaultConfig, **kwargs) -> NeroDriverDefault:
        """Nero CAN driver.

        Terminology
        -----------
        `flange`:
        - The mounting face / connection interface on the robotic arm's last link
          (mechanical tool interface).

        Common conventions
        ------------------
        `timeout` (for request/response style APIs):
        - `timeout < 0.0` raises ValueError.
        - `timeout == 0.0`: non-blocking; evaluate readiness once and return
          immediately.
        - `timeout > 0.0`: blocking; poll until ready or timeout expires.

        `joint_index`:
        - `joint_index == 255` means "all joints".

        `set_*` return semantics:
        - Many `set_*` APIs are ACK-only: True means the controller acknowledged the
          request.
          This does not strictly guarantee the setting is already applied.
        - Some `set_*` APIs additionally verify by reading back state; their
          docstrings will mention the verification method if applicable.
        """
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperCanDefaultConfig, **kwargs) -> PiperDriverDefault:
        """Piper CAN driver.

        Terminology
        -----------
        `flange`:
        - The mounting face / connection interface on the robotic arm's last link
          (mechanical tool interface).

        Common conventions
        ------------------
        `timeout` (for request/response style APIs):
        - `timeout < 0.0` raises ValueError.
        - `timeout == 0.0`: non-blocking; evaluate readiness once and return
          immediately.
        - `timeout > 0.0`: blocking; poll until ready or timeout expires.

        `joint_index`:
        - `joint_index == 255` means "all joints".

        `set_*` return semantics:
        - Many `set_*` APIs are ACK-only: True means the controller acknowledged the
          request.
          This does not strictly guarantee the setting is already applied.
        - Some `set_*` APIs additionally verify by reading back state; their
          docstrings will mention the verification method if applicable.
        """
        ...
    
    @classmethod
    @overload
    def create_arm(cls, config: PiperHCanDefaultConfig, **kwargs) -> PiperHDriverDefault:
        """PiperH CAN driver.

        Terminology
        -----------
        `flange`:
        - The mounting face / connection interface on the robotic arm's last link
          (mechanical tool interface).

        Common conventions
        ------------------
        `timeout` (for request/response style APIs):
        - `timeout < 0.0` raises ValueError.
        - `timeout == 0.0`: non-blocking; evaluate readiness once and return
          immediately.
        - `timeout > 0.0`: blocking; poll until ready or timeout expires.

        `joint_index`:
        - `joint_index == 255` means "all joints".

        `set_*` return semantics:
        - Many `set_*` APIs are ACK-only: True means the controller acknowledged the
          request.
          This does not strictly guarantee the setting is already applied.
        - Some `set_*` APIs additionally verify by reading back state; their
          docstrings will mention the verification method if applicable.
        """
        ...
    
    @classmethod
    @overload
    def create_arm(cls, config: PiperLCanDefaultConfig, **kwargs) -> PiperLDriverDefault:
        """PiperL CAN driver.

        Terminology
        -----------
        `flange`:
        - The mounting face / connection interface on the robotic arm's last link
          (mechanical tool interface).

        Common conventions
        ------------------
        `timeout` (for request/response style APIs):
        - `timeout < 0.0` raises ValueError.
        - `timeout == 0.0`: non-blocking; evaluate readiness once and return
          immediately.
        - `timeout > 0.0`: blocking; poll until ready or timeout expires.

        `joint_index`:
        - `joint_index == 255` means "all joints".

        `set_*` return semantics:
        - Many `set_*` APIs are ACK-only: True means the controller acknowledged the
          request.
          This does not strictly guarantee the setting is already applied.
        - Some `set_*` APIs additionally verify by reading back state; their
          docstrings will mention the verification method if applicable.
        """
        ...
    
    @classmethod
    @overload
    def create_arm(cls, config: PiperXCanDefaultConfig, **kwargs) -> PiperXDriverDefault:
        """PiperX CAN driver.

        Terminology
        -----------
        `flange`:
        - The mounting face / connection interface on the robotic arm's last link
          (mechanical tool interface).

        Common conventions
        ------------------
        `timeout` (for request/response style APIs):
        - `timeout < 0.0` raises ValueError.
        - `timeout == 0.0`: non-blocking; evaluate readiness once and return
          immediately.
        - `timeout > 0.0`: blocking; poll until ready or timeout expires.

        `joint_index`:
        - `joint_index == 255` means "all joints".

        `set_*` return semantics:
        - Many `set_*` APIs are ACK-only: True means the controller acknowledged the
          request.
          This does not strictly guarantee the setting is already applied.
        - Some `set_*` APIs additionally verify by reading back state; their
          docstrings will mention the verification method if applicable.
        """
        ...
