# TclRCX Bytecode Compiler
An extension to the [Tcl interpreter](https://www.tcl.tk/) to let you issue commands to the RCX interactively or compile bytecode programs, download, and run them


## Background
It was originally based on [TclRCX by Laurent Demailly](http://www.demailly.com/tcl/rcx/),
the bytecode compilation functions were made possible mainly by the excellent reverse
engineering and RCX bytecode documentation provided by [Kekoa Proudfoot on his RCX Internals web page](http://www.mralligator.com/rcx/).
If you are interested in using the RCX to its greatest potential, 
[Russell Nelson's Lego Mindstorms Internals web page](http://crynwr.com/lego-robotics/) is the place to go.


## Setup
Programming for the RCX with these Tcl commands lets you go well beyond the
capabilities of the Mindstorms programming environment.
Since this program outputs RCX bytecode as implemented by the Mindstorms
firmware, you will need to download the Lego firmware to your RCX before
downloading and running any of your bytecode programs.
To download firmware from platforms other than Windows, you will need to use
alternate tools, such as `firmdl` that is included with the brickOS-bibo project.


## Usage
To use this program you will need:
1. The [Tcl interpreter](https://www.tcl.tk/), which runs on Windows and many, many flavors of UNIX
2. The TclRCX Bytecode Compiler (`rcx.tcl`)
3. The [compiler documentation](http://web.archive.org/web/20010804084843/www.autobahn.org/~peterp/rcx/rcx-docs.html)


## Example
Example programs are in the “demos” folder;
“torbot.tcl” was written by Peter Pletcher to operate the Torbot model,
a Mindstorms challenge bot that drives around a table top without falling off.

To download that program to the RCX as program 5, for example, launch a Tcl shell and use commands such as the following:
```tcl
% source rcx.tcl
% connect
% source torbot.tcl
% beam 5
```

_Original website © 1998 Peter Pletcher_
