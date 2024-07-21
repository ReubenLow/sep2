# "Name of the System" Software Documentation


## "Name of the System" Description
The system consist of `key components` that is the 
  + Conveyer for Y Axis Movement
  + PrintHead/X Axis Carriage for X Axis Movement
  + Guiding Rod attached to printhead for Z Axis Movement
  + Pixy Camera for positional guidance
  + Keypad for program control

The software integrates all of the components, with the keypad being the control panel, X and Y axis fully automated with PixyCam as the main driver for positional movement and manual/human control for Z Axis.


## Pixy Camera For Positional Guidance

The `Pixy Camera` inteprets the conveyer belt based on image below. The pixy camera has two lines to create a `row` of objects along the `x-axis`.

These objects are pastries, with colour signature boxes on them.

![Alt text](URL or relative path to image)


## X Axis Print Head Movement

The X Axis Movement has the following **key** functions to guide the printhead to locations where the pastry is placed based on pixy camera.
    
+ `printHeadMovementRoutine_AUTO()`
+ `printHeadMovementRoutine_Manual()`
+ `backToHomePosition()`
+ `iniDelay()`
+ `xAxisMovement()`
+ `pressEveBot()`

## Y Axis Print Head Movement
The Y Axis Movement has the following **key** functions to guide and move the conveyer belt to move the tray of pastries along the Y axis.
+ `conveyerMovement(int z)`
+ `conveyerMovePastry()`


## Z Axis Print Head Movement
The Z Axis Movement has the following **key** functions to guide and move the conveyer belt to move the tray of pastries along the Y axis.
+ `moveZAxisUp()`
+ `moveZAxisDown()`


## Emphasis

*This text will be italic*  
_This will also be italic_

**This text will be bold**  
__This will also be bold__

_You **can** combine them_

## Lists

### Unordered

* Item 1
* Item 2
* Item 2a
* Item 2b

### Ordered

1. Item 1
2. Item 2
3. Item 3
    1. Item 3a
    2. Item 3b

## Images

![This is an alt text.](/image/sample.webp "This is a sample image.")

## Links

You may be using [Markdown Live Preview](https://markdownlivepreview.com/).

## Blockquotes

> Markdown is a lightweight markup language with plain-text-formatting syntax, created in 2004 by John Gruber with Aaron Swartz.
>
>> Markdown is often used to format readme files, for writing messages in online discussion forums, and to create rich text using a plain text editor.

## Tables

| Left columns  | Right columns |
| ------------- |:-------------:|
| left foo      | right foo     |
| left bar      | right bar     |
| left baz      | right baz     |

## Blocks of code

```
let message = 'Hello world';
alert(message);
```

## Inline code

This web site is using `markedjs/marked`.
