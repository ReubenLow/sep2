# "Name of the System" Software Documentation


## "Name of the System" Description
The system consist of `key components reflecting functionalities` that is the 
  + Pixy Camera for positional guidance
  + PrintHead/X Axis Carriage for X Axis Movement
  + Conveyer for Y Axis Movement
  + Guiding Rod attached to printhead for Z Axis Movement
  + Keypad for program control

The software integrates all of the components, with the keypad being the control panel, X and Y axis fully automated with PixyCam as the main driver for positional movement and manual/human control for Z Axis.


## Pixy Camera For Positional Guidance

The `Pixy Camera` inteprets the conveyer belt based on image below. These objects are pastries, with `colour signature` boxes on them. ***Based on the axes, towards right x value increases. As you go
down, y value increases.***

![image](https://github.com/user-attachments/assets/0a37ec5b-789d-44c6-a19a-3bd4714ff4cd)


The pixy camera has two lines to create a `row` of objects along the `X-axis`.

![image](https://github.com/user-attachments/assets/28489d34-fe56-4523-99c5-2de9eea9a570)

### Row Object

#### Creating a `Row` Reference Point

To guide X and Y Axis to move and print on which pastries, the program creates a `Row`, consist of `x coordinates` of pastries along the same row, a y coordinate `coordRow_Y` that serves as **a reference point to bring the `Row` to the `PRINTLINE`** and the number of pastries/objects in the `Row`.

```
typedef struct __row{
	uint16_t coordRow_Y;
	uint16_t numberOfRowObjects;
	uint16_t coordXArray[10];
}Row;
```

**The program creates a reference point for a `Row`, as described in the code snippet below.**

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

![image](https://github.com/user-attachments/assets/3d105be6-bf91-4f05-8790-1a99ae46733b)

The image above describes a row being created because it **meets the two conditions mentioned**.

![image](https://github.com/user-attachments/assets/b0bcb62c-8961-493a-af73-fce657c0e12b)

The image above shows that **no row is created** because it **does not meet the second condition**.

### Adding Pastries to the `Row` Object

Once `Row` reference point is selected i.e. closest to `PRINT_LINE`, the program now adds other pastries along the `Y axis` within a buffer set.

Taking `Row.coordRow_Y`, which was set as a reference point along the Y axis i.e. conveyer, the program looks for pastries with centre point within `Row.coordRow_Y` and `Row.coordRow_Y - 10`. The snippet
describes this pastry selection for the `Row` object.

```
	for(int i = 0; i< numberOfBlocks; ++i)//loop for the no objects
	{
		//since the lowest centrepoint is the reference for the row,
		//we account for objects within the gap
		//before the lowest centrepoint
		if(pastry[i].coordY >= (rowval - 10) && pastry[i].coordY <= rowval){
			row.coordXArray[i] = pastry[i].coordX;
			row.numberOfRowObjects++;
		}
	}
```

The following image describes that two crossaints are selected because their centrepoints along the y axis is between the `Row.coordRow_Y` and `Row.coordRow_Y - 10`.

![image](https://github.com/user-attachments/assets/046f05f3-0cde-4423-9ccf-0cb6a21168b1)

Hence, a row is created with the `Row.coordXArray[]` **filled with the centrepoints of pastries along the x axis.**
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
