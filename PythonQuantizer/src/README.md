# Python Quantize Tool

## How it Works
This tool quantizes `.synthSequence` to the nearest note value that the user specifies. It works by first asking the user for BPM of the piece, the note value to quantize to, and the file path of the `.synthSequence` file. The script then creates a grid of timestamps that correspond to the user-specified note value and BPM. Then, for each note in the `.synthSequence` file, it will align that note's starting value to the nearest timestamp on the grid. 

## Usage
1. Run `python quantize.py` in the command line
2. It will then prompt you for the BPM that you recorded your piece at. Enter the BPM.
	
    ```What BPM did you record at? 120```

3. It will then prompt you for the note you would like to quantize to. Enter the note.
	
    ```What note would you like to quantize to? (e.g. 1/4, 1/8, 1/16...) 1/4```
	
    *To quantize to the nearest bar, please type in `1/1` instead of `1`
4. It will then ask you for the file path of your `.synthSequence` file. Enter the file path.
	
    ```What is the file path of your note list? /Users/orimizrahi/School/cs190b/allolib_playground/tutorials/synthesis/bin/SuperSaw-data/super.synthSequence```

5. This will overwrite your old `.synthSequence` file, so your new file will have the same path.
