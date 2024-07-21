# "Name of the System" Software Documentation


## "Name of the System" Description
The system consist of `key components reflecting functionalities` that is the 
  + Conveyer for Y Axis Movement
  + PrintHead/X Axis Carriage for X Axis Movement
  + Guiding Rod attached to printhead for Z Axis Movement
  + Pixy Camera for positional guidance
  + Keypad for program control

The software integrates all of the components, with the keypad being the control panel, X and Y axis fully automated with PixyCam as the main driver for positional movement and manual/human control for Z Axis.


## Pixy Camera For Positional Guidance

The `Pixy Camera` inteprets the conveyer belt based on image below. These objects are pastries, with `colour signature` boxes on them.

![image](https://github.com/user-attachments/assets/0a37ec5b-789d-44c6-a19a-3bd4714ff4cd)


The pixy camera has two lines to create a `row` of objects along the `X-axis`.

![image](https://github.com/user-attachments/assets/28489d34-fe56-4523-99c5-2de9eea9a570)

### Row Object

To guide X and Y Axis to move and print on which pastries, the program creates a `Row`, consist of `x coordinates` of pastries along the same row, a y coordinate `coordRow_Y` that serves as a reference point to bring the `Row` to the `PRINTLINE` and the number of pastries/objects in the `Row`.

```
typedef struct __row{
	uint16_t coordRow_Y;
	uint16_t numberOfRowObjects;
	uint16_t coordXArray[10];
}Row;
```

The program creates a reference point for a `Row`, as described in the code snippet below.

The bottommost pastry i.e. pastry closest to the `PRINTLINE` is the reference point for a `Row`.

```
	for(int j = 0; j<numberOfBlocks; ++j)
	{
		//find the lowest centre point of an object within a "row" AND ONLY IF
		//The bottom width of the object is before the EVENT HORIZON
		//the lowest centre point of the object will be the main reference for a "row"
		
		if(pastry[j].coordY > rowval && pastry[j].bottomY<= EVENT_HORIZON) // EVENT_HORIZON is PRINTLINE - 10 pixels (Before PRINTLINE, hence subtract)
		{
			rowval = pastry[j].coordY;
		}

	}
```




The program first finds the pastry closes to the `PRINTLINE` i.e. bottommost pastry along the `Y Axis` by looking at **centrepoints** of these pastries. Finding the bottommost pastry among other pastries require `rowval` for comparison. The bottommost pastry is selected at the end of the **for loop**. This is the program **first condition**.

The **second condition** dictates that the pastry selected as the reference for `Row` must have its bottom width **before** the `PRINT_LINE` + 10 pixels line.

The images below descibes cases where `Row` is created based on conditions.

![image](https://github.com/user-attachments/assets/21595b5b-d5f8-4a2a-a392-afe5c4a93e7b)

The image above describes a row being created because it **meets the two conditions mentioned**.

![image](https://github.com/user-attachments/assets/a75389b7-3252-4dd9-ab67-384dc595e4b5)

The image above shows that **no row is created** because it **does not meet the second condition**.



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
+ `moveZAxis()`


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
