# Formatting
To make the code in this repo more consistent, formatting guidelines have been established, and
VSCode will be *very* mad at you if you break the formatting guidelines. Nearly everything is
handled for you by the [black](https://black.readthedocs.io/en/stable/the_black_code_style/index.html) 
code formatter.

However there are some things you will still have to worry about:

## Naming
Here are the naming formats and objects they should be used on
- snake_case
  - parameters, variables, file names
- UPPER_CASE_SNAKE
  - constants (variables whose values shouldn't change)
- camelCase
  - methods
- PascalCase
  - classes

## Private Classes, Methods, and Attributes
Python doesn't have access modifiers, but you can indicate to other developers not to touch specific
objects by prefixing their name with an underscore: `_myPrivateMethod()`. This goes the other way:
if something is prefixed with an underscore, don't use it unless you're 100% sure what it does.

## Type Hinting
Python isn't a strongly typed language, meaning anything can be any type. However, in this repo
some typing is enforced. This is just to make development easier, as typing let's VSCode generate
autofill, meaning less needs to be memorized and typed. This can also reduce the risk of thinking
something is one type, when it is another type.

There are only a few spots where you will need to type hint:

### Method Definitions
Methods will need their return type defined and the types of its parameters defined. 

`cls` and `self` will not need to be typed. `__init__()` should just return `None`.

Here is an example of a typed method:
```py
def moveArm(arm_id: int, target_position: float) -> ArmState:
    pass

def moveArms(arm_ids: list[int], target_positions: list[float]) -> list[Armstate]:
    pass
```

### Attributes
Most of the time VSCode can figure out the type of attributes, but sometimes it will need help.
Usually on optional/possibly `None` attributes and any collections like `list` or `dict`.

Here is how to type attributes:
```py
def __init__(self, manager: Manager, arms: list[Arm]) -> None:
    # VSCode can figure out the type itself
    self.height = 0.0
    self.manager = manager
    self.arms = arms

    # VSCode needs some help
    self.state: State | None = None
    self.arm_ids_to_arm: dict[int, Arm] = {}
```

## Comments
Please comment your code. We're begging you. Please.

## Documentation
Documentation for all classes, modules, and public methods is required! There aren't super strict
guidelines for what to write, but the next section contains a general overview of what's 
recommended.

> Remember to limit the length of your lines to 100 columns!

### Modules
Modules in Python are a complicated thing, but for the purposes of this document, just think of them
as the `.py` file you write your code in. 

In your documentation, include
- The general purpose of the module
- The class(es) within the modules
  - A brief overview of those classes

### Classes
In your documentation, include
- The purpose of the class
- Usage examples (if needed)
- What contexts the class should be used
- Anything your class assumes to be true

### Methods
Methods prefixed with an underscore do not need documentation.

In your documentation, include
- What your method does
- What each parameter is for
- What your method returns

### Attributes
Documentation for attributes isn't required, but may be helpful in some cases.

In your documentation, include
- What your attribute stores
- What the value of the attribute is used for
- What changing the value may do

## Example
All in all, a fully documented, type hinted, and formatted file should look like this:
```py
"""
This module contains classes relating to the operation and use of Bananas. It includes 
`Banana`, `BananaPortion`, and `BananaEating`. They represent a whole banana, a portion of a banana,
and the process of eating a banana respectively.
"""
class Banana:
    """
    `Banana` represents a whole banana that can be eaten in portions by `BananaEating`.
    """

    def __init__(self, portions: list[Banana]) -> None:
        """
        Initializes a Banana object.

        Parameters
        -----
        portions: list[Banana]
            The portions this banana represents.
        """

        # Don't use me! This is a private attribute
        self._portions = portions

    def getPortions(self) -> list[BananaPortion]:
        """
        Returns a list of `BananaPortion` containing the portions this banana represents.

        Returns
        ------
            A list of `BananPortion` containing the portions this banana represents.
        """
        return self._portions


class BananaPortion:
    """
    `BananaPortion` represents a portion of a whole banana. It can be eaten by `BananaEating`.
    """

    def __init__(self, banana: Banana) -> None:
        self.banana = banana


class BananaEating:
    """
    `BananaEating` represents the action of eating a banana. It is meant to be used alongside
    `BananaPortion`. This class is asynchronous, and thus should NOT be used in a `Node` thread.
    """
    
    def __init__(self, portion: BananaPortion) -> None:
        self.portion = portion
        """
        The portion portion of a banana this instance will eat.
        """

    # Doesn't need docs because it's marked as private!
    def _eatInternal(self) -> None:
        pass

    def eat(self) -> None:
        """
        Eats the portion of banana in the `portion` attribute of this class.
        """
        pass
```