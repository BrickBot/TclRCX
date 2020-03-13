# Tcl RCX Bytecode Compiler
#
# Copyright (c) 1998,
#   Laurent Demailly  mailto:L@Demailly.com       http://www.demailly.com/~dl/
#   Peter Pletcher    mailto:peterp@autobahn.org
# See the 'Artistic' LICENSE (http://language.perl.com/misc/Artistic.html)
# for terms, conditions, and in particular the DISCLAIMER OF ALL WARRANTIES.
# (If you'd like or need a license with other terms, don't hesitate
#  to contact the author)
#
# LEGO is a registered trademark of the LEGO Group, which does not sponsor,
# authorize, or endorse this work.
#
# This program was originally based on "Tcl RCX", by Laurent Demailly
#   http://www.demailly.com/tcl/rcx/    mailto:L@Demailly.com
# Most of what I know about how the RCX works was learned from:
#   - Kekoa Proudfoot's "RCX Internals" web page,
#     http://graphics.stanford.edu/~kekoa/rcx/
#   - Russell Nelson's "Lego Mindstorms Internals" web page,
#     http://www.crynwr.com/lego-robotics/
#   - Paul Haas' Perl program "talkrcx"
#     http://www.hamjudo.com/rcx/

#============================================================================
# Todo:
#
#   Remaining opcodes: 
#     20  Get Memmap
#     a4  Upload datalog
#     61  Delete task          40  Delete all tasks
#     c1  Delete subroutine    70  Delete all subroutines
#     75  Start firmware d/l   a5  Unlock firmware         65  Delete firmware
#     f7  Set message buffer   90  Clear Message buffer    b2  Send message
#
#   Test datalog feature
#
#   jmp should generate 'Branch always near' or 'Branch always far',
#     depending on the branch distance
#     This would require fixing up labels and offsets on the fly. Right.
#
#   Error checking!
#     Check values for given immediate data: must be short integer
#
#   Abort compilation on errors
#     Report error with source line number
#
#   Perhaps: sensor 2 reads light using percent
#
#   Have beam command split task code into blocks if needed
#

#============================================================================
#
# Communication code
#
namespace eval ::rcx {
	namespace export sendPacket getPacket debug_level flip code2packet
	namespace export connect disconnect
	#
	# Variables
	#

	# Serial port used for communication with RCX:
	#   device name and channel
	variable serial_port
	variable chan

	# Opcode bit flipping
	variable flip 0

	# Read timeout
	variable timedout

	# Debugging level:
	#	0	- quiet
	#	1	- comm timeouts, responses OK or not
	#	3	- bytecode sent/received, tower echo
	#	5	- detailed, including data bytes sent/received
	variable debug_level 0

	#
	# Platform dependent serial port device names:
	#
	switch $tcl_platform(platform) {
		"windows" {
			variable default_serial_port COM1:
		}
		"unix" {
			variable default_serial_port /dev/ttyS0
		}
		"macintosh" {
			# How do we name serial ports on a Macintosh?
			error "I don't know how to use serial ports on a Macintosh... yet"
		}
	}

	#
	# Open the serial port
	#
	proc connect { {port ""} } {
		variable default_serial_port
		variable serial_port
		variable chan
		variable status

		if [string length $port] {
			# Use specified port
			set serial_port $port
		} else {
			# Use default port
			set serial_port $default_serial_port
		}
		# Open port and set it up for RCX communications
		set chan [open $serial_port RDWR]
		fconfigure $chan -mode 2400,o,8,1 \
				-buffering none -translation binary -blocking false
		set status 0
		return
	}

	#
	# Close the serial port
	#
	proc disconnect {} {
		variable chan

		close $chan
		unset chan
	}

	#
	# Print debugging message
	#
	proc debug { level message } {
		variable debug_level
		if { $debug_level >= $level } {
			puts -nonewline $message ; flush stdout
		}
	}

	#
	# convert binary data to hex string
	#   "123" becomes "313233"
	#
	proc bin2hex { bin } {
		binary scan $bin H* hex
		return $hex
	}

	#
	# convert hex string to binary data
	#  "0x31 0x32 0x33" becomes "123"
	#
	proc hex2bin { hex } {
		# remove whitespace
		regsub -all "\[ \t\n\]+" $hex {} hex
		# remove "0x" prefixes
		regsub -all -nocase 0x $hex {} hex
		binary format H* $hex
	}

	#
	# convert byte value to hex string
	#  "49" becomes "31"
	#
	proc val2hex { val } {
		format "%02x" $val
	}

	# Alternately flip the 0x08 bit on opcodes
	proc flip { opcode } {
		variable flip
		if { $flip } {
			set flip 0
			return [val2hex [expr { 0x08 ^ "0x$opcode" }]]
		} else {
			set flip 1
			return $opcode
		}
	}

	# IR packets:
	#  header     bytecodes 0-n        checksum
	# /------\ /----------------------\ /----\
	# 55 FF 00 d0 ~d0 d1 ~d1 ... dn ~dn cc ~cc
	# Packet header is 55 FF 00
	# Each bytecode is followed by its one's complement
	# The checksum (8 bit sum of bytecodes) is followed by its complement
	#   "f7 12" becomes "55ff00f70812ed09f6"
	proc code2packet { bytecode } {
		set packet { 55 ff 00 }
		set checksum 0
		# Append each bytecode and complement to packet
		foreach byte $bytecode {
			lappend packet $byte
			set val 0x$byte
			lappend packet [val2hex [expr { 0xff ^ $val }]]
			incr checksum $val
		}
		# Apply mask for eight-bit checksum and append to packet
		set checksum [ expr { $checksum & 0xff } ]
		lappend packet [val2hex $checksum] [val2hex [expr { 0xff ^ $checksum}]]
		join $packet ""
	}

	# Convert a binary packet to the bytecode it represents
	proc packet2code { packet } {
		set code {}
		set len [string length $packet]
		# Check packet header
		if { [bin2hex [string range $packet 0 2]] != "55ff00" } {
			debug 1 "error: malformed packet [bin2hex $packet]\n"
			return {}
		}
		# Pick up the bytecodes from packet body, skipping checksums
		for { set i 3 } { $i < [expr { $len - 2 }] }  { incr i 2 } {
			lappend code [bin2hex [string index $packet $i]]
		}
		return $code
	}

	#
	# Create a packet from bytecode and send it to the RCX,
	# Then read and verify the tower echo
	#
	proc sendPacket { bytecode } {
		variable status
		variable chan
		variable timedout

		debug 5 "sendPacket: got $bytecode\n"

		# Construct packet to send
		set packet [code2packet $bytecode]
		set bindata [hex2bin $packet]
		set bindata_len [string length $bindata]

		# Eat any pending input and send packet
		catch { read $chan }
		puts -nonewline $chan $bindata
		flush $chan
		debug 5 "sent $packet\n"

		# Wait for the tower echo, or time out
		debug 5 "Reading $bindata_len echo: "
		set resp [getPacket $bindata_len]
		if { $timedout != 0 } {
			debug 5 "Echo read timed Out.\n"
			return 1
		}
		debug 5 "Got [bin2hex $resp]\n"
		if {[string compare $resp $bindata]} {
			debug 1 " error, invalid echo: [bin2hex $resp]\n"
			return 1
		} else {
			debug 3 "echo OK.\n"
			return 0
		}
	}

	# Wait for an expected response packet, or time out
	# Put the packet into named variable
	# Return -1 if the read timed out, 0 otherwise
	proc getResponse { resp_len name { timeout 4000 } } {
		variable chan
		variable timedout
		variable status
		upvar $name packet
		# Response binary data length = response codes * 2 + header + checksum
		set resp_bindata_len [ expr { $resp_len * 2 + 3 + 2 } ]
		debug 5 "Reading $resp_bindata_len resp: "
		set resp [getPacket $resp_bindata_len]
		debug 5 "Got [bin2hex $resp]\n"
		if { $timedout != 0 } {
			debug 1 "Response read timed out.\n"
			set status -1
		} else {
			set packet [packet2code $resp]
			set status 0
		}
	}

	# Read an expected response, or time out
	# Set variable timedout if the read times out
	proc getPacket { expected_len { timeout 4000 } } {
		variable chan
		variable timedout
		set resp ""
		set resp_len 0
		set timedout 0
		set timedout_after [ after $timeout { set rcx::timedout 1 } ]
		set header_found 0
		while { $timedout == 0 } {
			set w ""
			catch { set w [read $chan 1] }
			if { [string length $w] != 0 } {
				if { [bin2hex $w] == "55" } {
					set header_found 1
					debug 5 "<HDR>"
				}
				if { $header_found == 1 } {
					append resp $w
					incr resp_len [string length $w]
				}
				# See the data received
				debug 5 "<[bin2hex $w]>"
				if { $resp_len >= $expected_len } {
					after cancel $timedout_after
					break
				}
			}
			update
		}
		if { $timedout != 0 } {
			debug 5 "getPacket timed out."
		}
		return $resp
	}

	# Read from RCX channel forever, printing the bytes read
	# Stop this process by:  set rcx::timedout 2
	proc monitor { } {
		variable chan
		variable timedout

		# fconfigure stdin -blocking false
		set timedout 1
		while { $timedout != 2 } {
			# Keep tower power on: periodically send a byte
			if {$timedout == 1} {
				set timedout 0
				set timedout_after [ after 1007 { \
						puts -nonewline $rcx::chan [rcx::hex2bin aa] ; \
						flush $rcx::chan ; \
						set rcx::timedout 1
						puts -nonewline ";"
					} ]
			}
			# if {[read stdin 1] != ""} {break}

			set w ""
			catch { set w [read $chan 1] }
			if { [string length $w] != 0 } {
				# See the data received
				# Flag a possible 'keep-alive' character
				if {[bin2hex $w] == "aa"} {puts -nonewline ":"}
				# Put a newline before possible headers
				if {[bin2hex $w] == "55"} {puts ""}
				puts -nonewline "<[bin2hex $w]>"
			}
			update
		}
		# fconfigure stdin -blocking true
	}

}

#============================================================================
#
# Compilation code
#
##  <label_name>:                          Define a label
##  loop for <source>                      Begin loop
##  loop end                               End loop
##  begin task <n>                         Begin definition of task <n>
##  begin subroutine <n>                   Begin definition of subroutine <n>
##  end                                    End definition of task or sub
##  beam <prog>                            Beam program to RCX as prog <n>
#
namespace eval ::rcx {
	namespace export emit
	namespace export short2code source2code motors2code
	namespace export loop begin end beam

	#
	# Variables
	#

	# Program task and subroutine storage
	variable compiled_bytecode
	# Current bytecode being compiled
	variable current_compile
	# List of defined and unresolved labels
	variable defined_labels
	variable unresolved_labels

	# Looping controls
	variable next_loop_label
	variable curr_loop_level
	variable loop_labels

	# Emission mode:
	#   inspect      - Return bytecodes from command proc
	#   immediate    - Send bytecodes to RCX and interpret response
	#   deferred     - Compile bytecodes into a compiled_bytecode list
	variable mode immediate

	# Check for a numeric parameter in a specified range
	proc check_parameter { name val min max } {
		if {[catch { expr $val } v]} {
			error "I need a $name $min..$max, not \"$val\""
		}
		if { $v < $min || $v > $max } {
			error "Illegal $name $v, must be $min..$max"
		}
		return $v
	}

	# Emit bytecodes
	#   opcode, args   bytecodes to send
	#   resp_len       bytes of reply data
	#   resp_code      code to interpret reply data from $r
	proc emit { opcode {args ""} {resp_len 0} {resp_code {}} } {
		variable mode
		variable compiled_bytecode
		variable current_compile
		variable status
		if { $mode == "inspect" } {
			return "$opcode $args"
		} elseif { $mode == "immediate" } {
			transactCommand $opcode "$args" $resp_len $resp_code
			if { $status } {
				switch -- $status {
					-1  { puts "No response from RCX" }
					-2  { puts "Incorrect reply opcode from RCX" }
					default
					{ puts "An unknown error of type $status occurred" }
				}  
			} else { return }
		} else {
			# deferred
			append compiled_bytecode($current_compile) " $opcode $args"
			return
		}
	}

	# transactCommand        - send a bytecode command and verify the response
	proc transactCommand { opcode args resp_len resp_code } {
		variable status
		# send bytecode to RCX as a packet
		set opcode [flip $opcode]
		set status [sendPacket "$opcode $args"]
		# FIXME: check the status
		# read the response and interpret
		incr resp_len				;# for the opcode reply
		if { $resp_len > 0 } {
			if { [getResponse $resp_len r] } {
				return $status
			}
			debug 3 "RCX Response: $r\n"
			# Check the opcode reply against the opcode sent
			if { [lindex $r 0] == [val2hex [expr { 0xff ^ "0x$opcode" }]] } {
				set status 0
				if { [llength $resp_code] } {
					set r [lrange $r 1 end]
					eval $resp_code
				} else {
					puts "OK"
				}
			} else {
				set should [val2hex [expr { 0xff ^ "0x$opcode" }]]
				debug 1 "RCX Reply opcode wrong: was [lindex $r 0] but should be $should\n"
				set status -2
			}
		}
		return $status
	}

	# Download bytecode as a task or subroutine
	proc download_bytecode { type index code } {
		set blocknum 0
		# compute length and checksum
		set len [llength $code]
		set checksum 0
		foreach byte $code {
			set val 0x$byte
			incr checksum $val
		}
		set checksum [ expr { $checksum & 0xff } ]
		# send Start Task Download (op 25) or Start Subroutine Download (op 35)
		if { $type == "task" } {
			set op 25
		} elseif { $type == "sub" } {
			set op 35
		}
		emit $op "00 [short2code $index] [short2code $len]" 1 {
			set v [expr { "0x$r" } ]
			if { $v != 0 } {
				puts -nonewline "$type download error: "
				if { $v == 1 } {
					puts "Not enough memory for $len byte $type"
				} elseif { $v == 2 } {
					puts "Invalid $type index $index"
				} else {
					puts "error code $v"
				}
			} else {
				debug 3 "OK\n"
			}
		}
		# send Transfer Data (op 45) and data
		emit 45 "[short2code $blocknum] [short2code $len] $code [format %02x $checksum]" 1 {
			set v [expr { "0x$r" } ]
			if { $v != 0 } {
				puts -nonewline "$type download error: "
				if { $v == 3 } {
					puts "Checksum error, block $blocknum"
				} elseif { $v == 4 } {
					puts "Firmware checksum error, whatever that is..."
				} elseif { $v == 6 } {
					puts "Invalid or missing download start"
				} else {
					puts "error code $v"
				}
			} else {
				debug 3 "OK\n"
			}
		}
	}

	#
	# Data translation
	#

	proc code2short { code } {
		expr { "0x[lindex $code 1]" * 256 + "0x[lindex $code 0]" }
	}

	# Translate a value to a bytecode
	proc short2code { val } {
		set h [format %04hx $val]
		return "[string range $h 2 3] [string range $h 0 1]"
	}
	proc byte2code { val } {
		format "%02x" [expr { $val & 0xff } ]
	}

	# Translate a source specification into bytecode
	# Source types:
	#   source    arg     Description         Example Spec
	#   ------  -------   -----------------   ------------------------
	#     0      0..31    register            r0 r1 .. r31
	#     1      0..3     timer               timer 0  ..  timer 3
	#     2      n        immediate           12 999 -10
	#     3      0..2     motor state         motor A  motor B  motor C
	#  motor state result =  <0x07=power 0x08=fwd 0x40=off 0x80=on 0x00=float>
	#     4      n        random # 0..n       random n
	#     8      0        current program #   program
	#     9      0..2     sensors             sensor 1  sensor 2  sensor 3
	#    10      0..2     sensor type         sensorType 1  ..  sensorType 3
	#    11      0..2     out to sensor       sensorOut 1  ..  sensorOut 3
	#    12      0..2     raw sensor value    sensorRaw 1  ..  sensorRaw 3
	#    13      0..2     unknown
	#    14      0        minutes on watch    watch
	#    15      0        current message     message
	proc source2code { type args } {
		set sourceSpec [lindex $args 0]
		set source [lindex $sourceSpec 0]
		switch [llength $sourceSpec] {
			1 {
				# Single word: r0 program watch message immediate
				switch -regexp -- $source {
					^\[rR\]\[0-9\]+$ {
						# register spec:         r0 r1 .. r31
						regexp -nocase ^r(\[0-9\]+) $source match index
						if { $index > 31 } {
							error "Invalid register specifier $source, must be r0 - r31"
						}
						set r "00 [${type}2code $index]"
					}
					^program$ {
						# current program number:  program
						set r "08 [${type}2code 0]"
					}
					^watch$ {
						# minutes on watch:       watch
						set r "0e [${type}2code 0]"
					}
					^message$ {
						# current message:        message
						set r "0f [${type}2code 0]"
					}
					default {
						# immediate value
						# Check for a legal value
						if {[catch { expr $source } val]} {
							error "Invalid source spec \"$source\""
						}
						set r "02 [${type}2code $val]"
					}
				}
			}
			2 {
				# Word w/param: timer motor random sensor sensorType sensorRaw
				set index [lindex $sourceSpec 1]
				switch -- $source {
					timer {
						# timer spec:            timer 0  ..  timer 3
						if {$index < 0 || $index > 3} {
							error "Illegal timer number $index, must be 0..3"
						}
						set r "01 [${type}2code $index]"
					}
					motor {
						# motor spec:            motor A   motor B   motor C
						if { [regexp ^(\[ABCabc\]) $index match m] } {
							set index [string first [string toupper $m] "ABC"]
							set r "03 [${type}2code $index]"
						} else {
							error "Invalid motor specifier \"$source $index\""
						}
					}
					random {
						# random number 0..n:    random n
						set r "04 [${type}2code $index]"
					}
					sensor {
						# sensor value:          sensor 1   sensor 2   sensor 3
						if {$index < 1 || $index > 3} {
							error "Illegal sensor number $index, must be 1..3"
						}
						set r "09 [${type}2code [incr index -1]]"
					}
					sensorType {
						# sensor type:           sensorType 1  ..  sensorType 3
						if {$index < 1 || $index > 3} {
							error "Illegal sensor number $index, must be 1..3"
						}
						set r "0a [${type}2code [incr index -1]]"
					}
					sensorRaw {
						# sensor raw input:      sensorRaw 1  ..  sensorRaw 3
						if {$index < 1 || $index > 3} {
							error "Illegal sensor number $index, must be 1..3"
						}
						set r "0c [${type}2code [incr index -1]]"
					}
					default
					{ error "Unknown source specifier \"[join $sourceSpec]\"" }
				}
			}
			default
			{ error "Unknown source specifier \"[join $sourceSpec]\"" }
		}
		return $r
	}

	proc junk_source2code { type args } {
		if { [regexp -nocase ^r(\[0-9\]+) $source match index] } {
			# register spec:         r0 r1 .. r31
			if { $index > 31 } {
				error "Invalid register specifier $source, must be r0 - r31"
			}
			set r "00 [${type}2code $index]"
		} elseif { $source == "timer" } {
			# timer spec:            timer 0  ..  timer 3
			set index [lindex $sourceSpec 1]
			set r "01 [${type}2code $index]"
		} elseif { [regexp ^(-?\[0-9\]+) $source match val] } {
			# numeric value
			# This should be the default with an eval?
			set r "02 [${type}2code $val]"
		} elseif { $source == "motor" } {
			# motor spec:            motor A   motor B   motor C
			set index [lindex $sourceSpec 1]
			if { [regexp ^(\[ABCabc\]) $index match m] } {
				set index [string first [string toupper $m] "ABC"]
				set r "03 [${type}2code $index]"
			} else {
				error "Invalid motor specifier \"$source $index\""
			}
		} elseif { $source == "random" } {
			# random number 0..n:    random n
			set index [lindex $sourceSpec 1]
			set r "04 [${type}2code $index]"
		} elseif { $source == "program" } {
			# current program number:  program
			set r "08 00"
		} elseif { $source == "sensor" } {
			# sensor value:            sensor 1   sensor 2   sensor 3
			set index [lindex $sourceSpec 1]
			set r "09 [${type}2code [incr index -1]]"
		} elseif { $source == "sensorType" } {
			# sensor type:            sensorType 1  sensorType 2  sensorType 3
			set index [lindex $sourceSpec 1]
			set r "0a [${type}2code [incr index -1]]"
		} elseif { $source == "sensorOut" } {
			# sensor output:          sensorOut 1  sensorOut 2  sensorOut 3
			set index [lindex $sourceSpec 1]
			set r "0b [${type}2code [incr index -1]]"
		} elseif { $source == "sensorRaw" } {
			# sensor raw input:       sensorRaw 1  sensorRaw 2  sensorRaw 3
			set index [lindex $sourceSpec 1]
			set r "0c [${type}2code [incr index -1]]"
		} elseif { $source == "watch" } {
			# minutes on watch:       watch
			set r "0e 00"
		} elseif { $source == "message" } {
			# current message:        message
			set r "0f 00"
		} else {
			error "Unknown source specifier \"$source\""
		}
		return $r
	}

	# Translate a motor spec into bytecode
	# Motor spec contains characters [abcABC]
	proc motors2code { motors } {
		set m 0
		if { [string first "a" $motors]>=0 || [string first "A" $motors]>=0 } {
			set m [expr $m | 0x01]
		}
		if { [string first "b" $motors]>=0 || [string first "B" $motors]>=0 } {
			set m [expr $m | 0x02]
		}
		if { [string first "c" $motors]>=0 || [string first "C" $motors]>=0 } {
			set m [expr $m | 0x04]
		}
		return $m
	}

	proc interp_jmpoffset { type offset } {
		variable compiled_bytecode
		variable current_compile
		variable defined_labels
		variable unresolved_labels

		if { $type == "tbr" } {
			set field_pos 6
		} else {
			set field_pos 1
		}
		if { [catch { expr $offset } o] } {
			# offset is presumably a label name
			# Try to resolve the label, or add it to unresolved list
			set pc [llength $compiled_bytecode($current_compile)]
			if { [set i [lsearch $defined_labels [list $offset *]]] >= 0 } {
				# Get the offset of a defined label
				set o [lindex [lindex $defined_labels $i] 1]
				# Make offset relative to the opcode's offset field
				set o [ expr { $o - $pc - $field_pos } ]
			} else {
				# Add to unresolved_labels(name): {type code_index}
				lappend unresolved_labels($offset) [list $type $pc]
				set o 0
			}
		} else {
			# offset is an explicit relative offset
			# make it relative to this opcode
			incr o [expr -$field_pos]
		}
	}

	#
	# Meta-commands
	#

	proc label { name } {
		variable mode
		variable compiled_bytecode
		variable current_compile
		variable defined_labels
		variable unresolved_labels

		# Labels only make sense in deferred mode
		if { $mode != "deferred" } {
			set msg "Labels only make sense in deferred mode\n"
			append msg "  use begin task <n> first"
			error $msg
		}
		# Check if this label is already defined
		if { [lsearch $defined_labels [list $name *]] >= 0 } {
			error "label \"$name\" is already defined"
		}
		set pc [llength $compiled_bytecode($current_compile)]
		lappend defined_labels [list $name $pc]
		# Check for unresolved references to this label, and resolve them
		if { [info exists unresolved_labels($name)] } {
			foreach ref $unresolved_labels($name) {
				set patch [lindex $ref 1]
				switch -- [lindex $ref 0] {
					jmpnear {
						# Point to the offset field
						incr patch 1
						set o [expr $pc - $patch]
						# Patch the existing bytecode
						if { $o < 0 } {
							set direction 0x80
							set o [expr -$o]
						} else {
							set direction 0
						}
						# FIXME: Check that offset is in range (<= 0x7f)
						set o [expr $o | $direction]
						set compiled_bytecode($current_compile) [lreplace $compiled_bytecode($current_compile) $patch $patch [format %02x $o]]
					}
					jmp {
						# Point to the offset field
						incr patch 1
						set o [expr $pc - $patch]
						# Patch the existing bytecode
						if { $o < 0 } {
							set direction 0x80
							set o [expr -$o]
						} else {
							set direction 0
						}
						set e [expr $o >> 7]
						set o [expr $o & 0x7f]
						set o [expr $o | $direction]
						set compiled_bytecode($current_compile) [lreplace $compiled_bytecode($current_compile) $patch [expr $patch + 1] [format %02x $o] [format %02x $e]]
					}
					tbr {
						# Point to the offset field
						incr patch 6
						set o [expr $pc - $patch]
						# Patch the existing bytecode
						set o [short2code $o]
						set compiled_bytecode($current_compile) [lreplace $compiled_bytecode($current_compile) $patch [expr $patch + 1] [lindex $o 0] [lindex $o 1]]
					}
					loop {
						# Point to the offset field
						incr patch 1
						set o [expr $pc - $patch]
						# Patch the existing bytecode
						set o [short2code $o]
						set compiled_bytecode($current_compile) [lreplace $compiled_bytecode($current_compile) $patch [expr $patch + 1] [lindex $o 0] [lindex $o 1]]
					}
						
				}
			}
			unset unresolved_labels($name)
		}
	}

	# Handle reasonable looping structures:
	#   loop for <n>  [setloop <n>] [label _loopNb] [decloop <--n < 0> _loopNe]
	#     code...
	#   loop end      [jmp _loopNb] [label _loopNe]
	# Using:
	#   setloop  82 Set Loop Counter <source 1,2,4:timer,immediate,random>
	#   decloop  92 Dec Loop Counter and Branch
	#               [branch forward only if counter < 0]
	proc loop { command args } {
		variable next_loop_label
		variable curr_loop_level
		variable loop_labels
		switch -- $command {
			for {
				# loop for <source>
				set s [source2code byte $args]
				set stype [lindex $s 0]
				if { $stype == "01" || $stype == "02" || $stype == "04" } {
					set n $next_loop_label
					# Write code:
					# [setloop <s>] [label _loopNb] [decloop <--s < 0> _loopNe]
					emit 82 "$s" -1
					label _loop${n}b
					interp_jmpoffset loop _loop${n}e
					set loop_labels($curr_loop_level) $n
					incr next_loop_label 1
					incr curr_loop_level 1
					emit 92 "00 00" -1
				} else {
					error "Looping for source \"$args\" not allowed"
				}
			}
			end {
				# loop end
				if {$curr_loop_level < 1} {
					error "No matching \"loop for\""
				}
				incr curr_loop_level -1
				set n $loop_labels($curr_loop_level)
				# Write code:
				# [jmp _loopNb] [label _loopNe]
				jmp _loop${n}b
				label _loop${n}e
			}
			default {
				set msg    "Usage is: loop for <source>\n"
				append msg "      or: loop end"
				error $msg
			}
		}
	}

	# Start deferred compilation into compiled_bytecode({task|sub},n)
	#   begin task <n>
	#   begin subroutine <n>
	proc begin { what which } {
		variable mode
		variable compiled_bytecode
		variable current_compile
		variable defined_labels
		variable unresolved_labels
		variable next_loop_label
		variable curr_loop_level
		variable loop_labels

		switch -- $what {
			task {
				if { $which < 0 || $which > 9 } {
					error "Invalid task number $which, must be 0..9"
				}
				set current_compile "task,$which"
			}
			sub  -
			subroutine {
				if { $which < 0 || $which > 7 } {
					error "Invalid subroutine number $which, must be 0..7"
				}
				set current_compile "sub,$which"
			}
			default {
				set msg    "Usage is: begin task <n>\n"
				append msg "      or: begin subroutine <n>\n"
				append msg "  where n is task number 0..9 or subroutine number 0..7"
				error $msg
			}
		}
		# Set up for compilation of a new section of code
		set mode deferred
		# Maybe issue a warning here if overwriting existing code
		set compiled_bytecode($current_compile) {}
		set defined_labels {}
		array set unresolved_labels {}
		set next_loop_label 0
		set curr_loop_level 0
		array set loop_labels {}
	}

	# end             - stop deferred compilation and check labels, etc.
	proc end {} {
		variable mode
		variable unresolved_labels

		if {[array size unresolved_labels]} {
			set msg "Some labels were left undefined:"
			foreach l [array names unresolved_labels] {
				append msg "\n  $l"
			}
			error $msg
		}
		set mode immediate
		return
	}

	# beam <prog>     - send all compiled bytecode to the RCX as program <prog>
	proc beam { prog } {
		variable status
		variable mode
		variable compiled_bytecode
		variable chan

		# Do an implicit end
		if { $mode == "deferred" } {
			end
		}
		# Open serial connection if needed
		if { ! [info exists chan] } {
			connect
		}
		# Ping the RCX and check for answer
		puts -nonewline "ping..."
		ping
		if {$status} { debug 3 "ping failed\n" ; return }
		# Set RCX program number
		puts -nonewline "program $prog..."
		program $prog
		if {$status} { debug 3 "setting program $prog failed\n" ; return }
		# Send each defined task
		for {set i 0} {$i <= 9} {incr i} {
			if { [catch { set compiled_bytecode(task,$i) } code] == 0 } {
				puts "sending task $i"
				download_bytecode task $i $code
			}
		}
		# Send each defined subroutine
		for {set i 0} {$i <= 7} {incr i} {
			if { [catch { set compiled_bytecode(sub,$i) } code] == 0 } {
				puts "sending subroutine $i"
				download_bytecode sub $i $code
			}
		}
		debug 3 "Done beaming.\n"
	}

}

#============================================================================
#
# RCX commands
#
#--------------------------
# By Opcode:
#
##  ping                               10  See if RCX responds
##  rget <bsource>                     12  Get value of <bsource>
##  motor <motors> power <bsource>     13  Set power of <motors> to <bsource>
##  rset <register> <source>           14  register = <source>
##  getVersion                         15  Get ROM and firmware versions
##  call <sub>                         17  Call subroutine <sub> (0..7)
#   getMemmap                          20  Get memory map
##  motor <motors> <on|off|float>      21  Set motor on/off
##  watch <hours> <minutes>            22  Set time on watch
##  tone <freq> <duration>             23  Play tone <freq> for <duration>
##  calc <register> + <source>         24  register += <source>
#<  ???                                25  Start task download
##  jmpnear <offset>                   27  Branch Always Near
##  getBattery                         30  Get battery voltage
##  transmitter <short|long>           31  Set transmitter range
##                                     32  Set sensor type
##  sensor <1|2|3> type <raw|touch|temp|light|rotation>
##  display <source>                   33  Set display to show device <source>
##  calc <register> - <source>         34  register -= <source>
#<  ???                                35  Start subroutine download
#   delete all tasks                   40  Delete all tasks
##                                     42  Set sensor mode
##  sensor <1|2|3> uses <raw|bool|edge|pulse|percent|deg_c|deg_f|angle> <slope>
##  wait <source>                      43  Wait <source> 100ths of a second
##  calc <register> / <source>         44  register /= <source>
#<  ???                                45  Transfer data
##  stop                               50  Stop all tasks
##  beep <sound>                       51  Play sound number <sound> (0..5)
##  datalog size <size>                52  Set datalog size to <size>
##  calc <register> * <source>         54  register *= <source>
##  power off                          60  Power off
#   delete task <task>                 61  Delete task <task>
##  datalog add <bsource>              62  Next Datalog entry = <bsource>
##  calc <register> sgn <source>       64  register = sgn(<source>) [-1,0,1]
#   delete firmware { byte key[5] }    65  Delete firmware
#   delete all subroutines             70  Delete all subroutines
##  start <task>                       71  Start task <task>
##  jmp <offset>                       72  Branch Always Far
##  calc <register> abs <source>       74  register = abs(<source>)
#   ???                                75  Start firmware download
##  stop <task>                        81  Stop task <task>
#<  setloop <source>                   82  Set loop counter to <source>
##  calc <register> and <source>       84  register &= <source>
##  message clear                      90  Clear Message buffer
##  program <prog>                     91  Set program number to <prog>
#<  decloop <uoffset>                  92  Decrement loop counter and branch
##  calc <register> or <source>        94  register |= <source>
##                                     95  Test and branch if <= >= == !=
##  tbr { <bsource> [<=|>=|==|!=] <source> } <offset>
##  clear timer <timer>                a1  Clear timer number <timer>
#   datalog upload <first> <count>     a4  Upload datalog
#   ??? <key>                          a5  Unlock firmware
##  power delay <minutes>              b1  Set power-down delay to <minutes>
##  message send <bsource>             b2  Send message <bsource>
#   delete subroutine <sub>            c1  Delete subroutine <sub>
##  sensor <1|2|3> clear               d1  Clear Sensor value
##  clear sensor <1|2|3>               d1  Clear Sensor value (also)
##  motor <motors> <forward|reverse|flip>  e1  Set motor direction
##  ret                                f6  Return from subroutine
##  message set <n>                    f7  Set message buffer to <n>
#
#--------------------------
# By Function:
#
# I/O functions:
#
##  ping                               10  See if RCX responds
##  transmitter <short|long>           31  Set transmitter range
##  rget <bsource>                     12  Get value of <bsource>
##  getVersion                         15  Get ROM and firmware versions
#   getMemmap                          20  Get memory map
##  getBattery                         30  Get battery voltage
##  watch <hours> <minutes>            22  Set time on watch
##  display <source>                   33  Set display to show device <source>
##  tone <freq> <duration>             23  Play tone <freq> for <duration>
##  beep <sound>                       51  Play sound number <sound> (0..5)
#
#<  ???                                25  Start task download
#<  ???                                35  Start subroutine download
#   ???                                75  Start firmware download
#   ??? <key>                          a5  Unlock firmware
#<  ???                                45  Transfer data
#
##  datalog size <size>                52  Set datalog size to <size>
##  datalog add <bsource>              62  Next Datalog entry = <bsource>
#   datalog upload <first> <count>     a4  Upload datalog
#
##  message send <bsource>             b2  Send message <bsource>
##  message set <n>                    f7  Set message buffer to <n>
##  message clear                      90  Clear message buffer
#
# Utility functions:
#
##  wait <source>                      43  Wait <source> 100ths of a second
##  clear timer <timer>                a1  Clear timer number <timer>
##  power off                          60  Power off
##  power delay <minutes>              b1  Set power-down delay to <minutes>
#
# Motor and Sensor functions:
#
##  motor <motors> <forward|reverse|flip>  e1  Set motor direction
##  motor <motors> power <bsource>     13  Set power of <motors> to <bsource>
##  motor <motors> <on|off|float>      21  Set motor on/off
##
##                                     42  Set sensor mode
##  sensor <1|2|3> uses <raw|bool|edge|pulse|percent|deg_c|deg_f|angle> <slope>
##                                     32  Set sensor type
##  sensor <1|2|3> reads <raw|touch|temp|light|rotation>
##  sensor <1|2|3> clear               d1  Clear Sensor value
##  clear sensor <1|2|3>               d1  Clear Sensor value
#
# Math functions:
#
##  rset <register> <source>           14  var(index) = <source>
##  calc <register> + <source>         24  register += <source>
##  calc <register> - <source>         34  register -= <source>
##  calc <register> / <source>         44  register /= <source>
##  calc <register> * <source>         54  register *= <source>
##  calc <register> sgn <source>       64  register = sgn(<source>) [-1,0,1]
##  calc <register> abs <source>       74  register = abs(<source>)
##  calc <register> and <source>       84  register &= <source>
##  calc <register> or <source>        94  register |= <source>
#
#     calc <register> <op> <source>      register = register <op> source
#               <op> is:  = + - / * sgn abs and or
#     signed numbers are 2's complement:
#       65535 is -1   65534 is -2   sgn(32767) = 1   sgn(32768) = -1
#     integer division
#     division and multiplication are signed
#     7 and 5 = 5
#     7 or  5 = 7
#
# Flow of control:
#
##  jmpnear <offset>                   27  Branch Always Near
##  jmp <offset>                       72  Branch Always Far
##                                     95  Test and branch if <= >= == !=
##  tbr { <bsource> [<=|>=|==|!=] <source> } <offset>
#
##  call <sub>                         17  Call subroutine <sub> (0..7)
##  ret                                f6  Return from subroutine
#   delete subroutine <sub>            c1  Delete subroutine <sub>
#   delete all subrouines              70  Delete all subroutines
#
##  start <task>                       71  Start task <task>
##  stop <task>                        81  Stop task <task>
##  stop                               50  Stop all tasks
#   delete all tasks                   40  Delete all tasks
#   delete task <task>                 61  Delete task <task>
#
#   delete firmware { byte key[5] }    65  Delete firmware
##  program <prog>                     91  Set program number to <prog>
#

# Beep sounds:
#  0  pip
#  1  mee-meep
#  2  arpeggio down
#  3  arpeggio up
#  4  buzz
#  5  fast arpeggio up

namespace eval ::rcx {
	namespace export ping transmitter rget getVersion getBattery display watch
	namespace export tone beep datalog message
	namespace export wait power motor sensor rset calc
	namespace export jmpnear jmp tbr call ret start stop program

	# sensor modes
	array set sensor_modes {
		raw     0
		bool    1
		edge    2
		pulse   3
		percent 4
		deg_c   5
		deg_f   6
		angle   7
	}
	# sensor types
	array set sensor_types {
		raw      0
		touch    1
		temp     2
		light    3
		rotation 4
	}

	# I/O functions:
	proc ping {} {
		emit 10
	}
	proc transmitter { range } {
		switch -- $range {
			short {
				set r 0
			}
			long {
				set r 1
			}
			default {
				error "Transmitter range must be \"short\" or \"long\", not $range"
			}
		}
		emit 31 "[format %02x $r]"
	}
	proc rget { args } {
		# Sources 2 (immediate) and 4 (random) not allowed
		set src1 [source2code byte $args]
		if {[lindex $src1 0] == "02" ||
			[lindex $src1 0] == "04"} {
			error "Source of type [lindex $src1 0] not allowed"
		}
		emit 12 "$src1" 2 {
			set v [code2short $r]
			if {$v > 32767} {
				# report signed value as well
				set sv [expr 65536 - $v]
				puts "value is $v (-$sv)"
			} else {
				puts "value is $v"
			}
		}
	}
	proc getVersion {} {
		emit 15 "01 03 05 07 0b" 8 {
			set romMajor [expr { "0x[lindex $r 0]"*256 + "0x[lindex $r 1]" } ]
			set romMinor [expr { "0x[lindex $r 2]"*256 + "0x[lindex $r 3]" } ]
			set frmMajor [expr { "0x[lindex $r 4]"*256 + "0x[lindex $r 5]" } ]
			set frmMinor [expr { "0x[lindex $r 6]"*256 + "0x[lindex $r 7]" } ]
			puts "Rom Version $romMajor.$romMinor"
			puts "Firmware Version $frmMajor.$frmMinor"
		}
	}
	proc getBattery {} {
		emit 30 {} 2 {
			set v [expr { [code2short $r] / 1000.0 } ]
			puts "battery voltage is $v volts"
		}
	}
	proc watch { hours minutes } {
		set h [check_parameter "hours" $hours 0 23]
		set m [check_parameter "minutes" $minutes 0 59]
		emit 22 "[format %02x $h] [format %02x $m]"
	}
	proc display { args } {
		emit 33 "[source2code short $args]"
	}
	proc tone { freq duration } {
		set f [check_parameter "frequency(in Hz)" $freq 0 65535]
		set d [check_parameter "duration(in sec/100)" $duration 0 65535]
		emit 23 "[short2code $f] [format %02x $d]"
	}
	proc beep { sound } {
		set s [check_parameter "sound number" $sound 0 5]
		emit 51 "[format %02x $s]"
	}
	proc datalog { command args } {
		switch -- $command {
			size {
				set s [check_parameter "datalog size" [lindex $args 0] 0 65535]
				emit 52 "[short2code $s]" 1 {
					if {$r == 1} {
						set msg    "The RCX does not have enough free memory\n"
						append msg "to allocate a $s byte datalog"
						error $msg
					}
				}
			}
			add {
				set source [source2code byte $args]
				set sourceType [lindex $source 0]
				if {$sourceType == "00" || $sourceType == "01" ||
					$sourceType == "09" || $sourceType == "0e"} {
						emit 62 "$source" 1 {
							if {$r == 1} {
								error "Datalog is full"
							}
						}
				} else {
					error "Source type $sourceType \"$args\" not allowed."
				}
			}
			default {
				set msg    "Usage is: datalog size <size>\n"
				append msg "          datalog add <bsource>\n"
				append msg "          datalog upload <first> <count>"
				error $msg
			}
		}
	}
	proc message { command args } {
		switch -- $command {
			send {
				# Send 'message set n' to other RCX's
				# Only sources 0 (register) and 2 (immediate) allowed
				set src [source2code byte $args]
				if {[lindex $src 0] == "00" || [lindex $src 0] == "02"} {
					emit b2 "$src" -1
				} else {
					error "Source of type [lindex $src 0] not allowed"
				}
			}
			set {
				# Set message buffer to a value 0-255
				set s [check_parameter "message number" [lindex $args 0] 0 255]
				emit f7 "[byte2code $s]" -1
			}
			clear {
				# Clear message buffer
				emit 90 {}
			}
			default {
				set msg    "Usage is: message send <bsource>\n"
				append msg "          message set <n>\n"
				append msg "          message clear"
				error $msg
			}
		}
	}
	# Utility functions:
	proc wait { args } {
		emit 43 "[source2code short $args]" -1
	}
	proc clear { what which } {
		switch -- $what {
			sensor {
				sensor $which clear
			}
			timer {
				set t [check_parameter "timer number" $which 0 3]
				emit a1 "[format %02x $t]"
			}
			default {
				set msg    "Usage is: clear sensor <n>\n"
				append msg "          clear timer <n>"
				error $msg
			}
		}
	}
	proc power { command {minutes 15} } {
		set m [check_parameter "number of minutes" $minutes 0 59]
		switch -- $command {
			off     { emit 60 {} -1 }
			delay   { emit b1 "[format %02x $m]" }
			default { error "Unknown power command $command" }
		}
	}
	# Motor and Sensor functions:
	proc motor { {motors "ABC"} {command "float"} args } {
		# FIXME: check arguments: power source 1,2,4  power level 0..7
		switch -- $command {
			reverse
			{ emit e1 "[format %02x [expr [motors2code $motors] | 0x00]]" }
			forward
			{ emit e1 "[format %02x [expr [motors2code $motors] | 0x80]]" }
			flip
			{ emit e1 "[format %02x [expr [motors2code $motors] | 0xC0]]" }
			power
			{ emit 13 "[format %02x [motors2code $motors]] [source2code byte $args]" }
			float
			{ emit 21 "[format %02x [expr [motors2code $motors] | 0x00]]" }
			off
			{ emit 21 "[format %02x [expr [motors2code $motors] | 0x40]]" }
			on
			{ emit 21 "[format %02x [expr [motors2code $motors] | 0x80]]" }
			default
			{ error "Invalid motor command $command" }
		}
	}
	proc sensor { sensor command args } {
		variable sensor_types
		variable sensor_modes
		set s [check_parameter "sensor number" $sensor 1 3]
		incr s -1
		switch -- $command {
			reads {
				if {[set type [lindex $args 0]] == ""} {
					error "I need a sensor type"
				}
				set t [array get sensor_types $type]
				if {[set t [lindex $t 1]] == ""} {
					error "Sensors can't read \"$type\""
				}
				emit 32 "[format %02x $s] [format %02x $t]"
			}
			uses {
				if {[set mode [lindex $args 0]] == ""} {
					error "I need a sensor mode"
				}
				set m [array get sensor_modes $mode]
				if {[set m [lindex $m 1]] == ""} {
					error "Sensors can't read using \"$mode\""
				}
				if {[set slope [lindex $args 1]] == ""} {
					set slope 0					;# Default slope
				}
				set sl [check_parameter "sensor slope" $slope 0 31]
				set m [expr { $sl | ($m << 5) } ]
				emit 42 "[format %02x $s] [format %02x $m]"
			}
			clear {
				emit d1 "[format %02x $s]"
			}
			default {
				error "Usage is: sensor <n> {reads|uses|clear}..."
			}
		}
	}
	# Math functions:
	proc rset { reg args } {
		# Get register spec r0 r1 .. r31
		if { [regexp -nocase "r(\[0-9\]+)" $reg match rindex] } {
			set index $rindex
		} else {
			error "Invalid register specifier $reg, must be r0 .. r31"
		}
		if { $index > 31 } {
			error "Invalid register specifier $reg, must be r0 .. r31"
		}
		emit 14 "[format %02x $index] [source2code short $args]"
	}
	proc calc { reg op args } {
		# Get register spec r0 r1 .. r31
		if { [regexp -nocase "r(\[0-9\]+)" $reg match rindex] } {
			set index $rindex
		} else {
			error "Invalid register specifier $reg, must be r0 .. r31"
		}
		if { $index > 31 } {
			error "Invalid register specifier $reg, must be r0 .. r31"
		}
		# Get operator
		array set operators {
			=       14
			+       24
			-       34
			/       44
			times   54
			sgn     64
			abs     74
			and     84
			or      94
		}
		# Hack around the globbing behavior of "*"
		if {$op == "*"} {set op times}
		set opcode [array get operators $op]
		if {[set opcode [lindex $opcode 1]] == ""} {
			error "Unknown operator \"$op\""
		}
		set src [source2code short $args]
		# Check for illegal source types
		if {$opcode != "14"} {
			if {[lindex $src 0] != "00" && [lindex $src 0] != "02"} {
				error "Source of type [lindex $src 0] not allowed for operator \"$op\""
			}
		}
		emit $opcode "[format %02x $index] $src"
	}
	# Flow of control:
	proc jmpnear { offset } {
		# Evaluate the given offset
		set o [interp_jmpoffset jmpnear $offset]

		# Fold in the direction bit
		if { $o < 0 } {
			set direction 0x80
			set o [expr -$o]
		} else {
			set direction 0
		}
		# Check that offset is in range
		if { $o > 0x7f } {
			error "jmpnear offset turned out to be $o, you need to use jmp"
		}
		set o [expr $o | $direction]
		emit 27 "[format %02x $o]" -1
	}
	proc jmp { offset } {
		# Evaluate the given offset
		set o [interp_jmpoffset jmp $offset]
		# Calculate offset and extension for 'Branch always far'
		if { $o < 0 } {
			set direction 0x80
			set o [expr -$o]
		} else {
			set direction 0
		}
		set e [expr $o >> 7]
		set o [expr $o & 0x7f]
		set o [expr $o | $direction]
		emit 72 "[format %02x $o] [format %02x $e]" -1
	}
	proc tbr { condition {offset 0} } {
		# Note: source 2 comes first, so you can say { r0 == 10 } e.g.
		if {[lindex $condition 1] == "<=" ||
			[lindex $condition 1] == ">=" ||
			[lindex $condition 1] == "==" ||
			[lindex $condition 1] == "!="} {
				set sourceSpec2 [lindex $condition 0]
				set op [lindex $condition 1]
				set sourceSpec1 [lrange $condition 2 end]
		} elseif { [lindex $condition 2] == "<=" ||
					[lindex $condition 2] == ">=" ||
					[lindex $condition 2] == "==" ||
					[lindex $condition 2] == "!="} {
						set sourceSpec2 [lrange $condition 0 1]
						set op [lindex $condition 2]
						set sourceSpec1 [lrange $condition 3 end]
		} else {
			error "I couldn't understand the condition expression $condition"
		}
		set src1 [source2code short $sourceSpec1]
		set src2 [source2code byte $sourceSpec2]
		# Check for illegal source types
		if {[lindex $src1 0] == "04" ||
			[lindex $src1 0] == "08"} {
			error "Source 1 of type [lindex $src1 0] not allowed"
		}
		if {[lindex $src2 0] == "02" ||
			[lindex $src2 0] == "04" ||
			[lindex $src1 0] == "08" } {
			error "Source 2 of type [lindex $src1 0] not allowed"
		}
		switch -- $op {
			<= { set op 0 }
			>= { set op 1 }
			!= { set op 2 }
			== { set op 3 }
		}
		# Pack up arguments
		set opsrc1 [expr { [lindex $src1 0] | ($op << 6) } ]
		set opsrc1 [byte2code $opsrc1]
		set arg2 [lindex $src2 1]
		set src2 [lindex $src2 0]
		set arg1 [lrange $src1 1 2]
		# Evaluate the offset
		set o [interp_jmpoffset tbr $offset]

		emit 95 "$opsrc1 $src2 $arg1 $arg2 [short2code $o]"
	}
	proc call { sub } {
		set s [check_parameter "subroutine number" $sub 0 7]
		emit 17 "[format %02x $s]" -1
	}
	proc ret {} {
		emit f6 {} -1
	}
	proc start { task } {
		set t [check_parameter "task number" $task 0 9]
		emit 71 "[format %02x $t]"
	}
	proc stop { {task ""} } {
		if {[llength $task]} {
			set t [check_parameter "task number" $task 0 9]
			emit 81 "[format %02x $t]"
		} else {
			emit 50
		}
	}
	proc program { prog } {
		set p [check_parameter "program number" $prog 1 5]
		incr p -1
		emit 91 "[format %02x $p]"
	}
}

# unknown -- RCX version
# This procedure is called when a Tcl command is invoked that doesn't
# exist in the interpreter.  It takes the following steps to make the
# command available:
#
#   0. If the command matches {^[a-zA-Z_]*:$} create an RCX code label
#      by calling rcx::label $name
#	1. See if the autoload facility can locate the command in a
#	   Tcl script file.  If so, load it and execute it.
#	2. If the command was invoked interactively at top-level:
#	    (a) see if the command exists as an executable UNIX program.
#		If so, "exec" the command.
#	    (b) see if the command requests csh-like history substitution
#		in one of the common forms !!, !<number>, or ^old^new.  If
#		so, emulate csh's history substitution.
#	    (c) see if the command is a unique abbreviation for another
#		command.  If so, invoke the command.
#
# Arguments:
# args -	A list whose elements are the words of the original
#		command, including the command name.

proc unknown args {
    global auto_noexec auto_noload env unknown_pending tcl_interactive
    global errorCode errorInfo

    # Save the values of errorCode and errorInfo variables, since they
    # may get modified if caught errors occur below.  The variables will
    # be restored just before re-executing the missing command.

    set savedErrorCode $errorCode
    set savedErrorInfo $errorInfo
    set name [lindex $args 0]

	if [regexp {^([a-zA-Z0-9_]*):$} $name dummy label_name] {
		return [rcx::label $label_name]
	}

    if ![info exists auto_noload] {
	#
	# Make sure we're not trying to load the same proc twice.
	#
	if [info exists unknown_pending($name)] {
	    return -code error "self-referential recursion in \"unknown\" for command \"$name\"";
	}
	set unknown_pending($name) pending;
	set ret [catch {auto_load $name} msg]
	unset unknown_pending($name);
	if {$ret != 0} {
	    return -code $ret -errorcode $errorCode \
		"error while autoloading \"$name\": $msg"
	}
	if ![array size unknown_pending] {
	    unset unknown_pending
	}
	if $msg {
	    set errorCode $savedErrorCode
	    set errorInfo $savedErrorInfo
	    set code [catch {uplevel 1 $args} msg]
	    if {$code ==  1} {
		#
		# Strip the last five lines off the error stack (they're
		# from the "uplevel" command).
		#

		set new [split $errorInfo \n]
		set new [join [lrange $new 0 [expr [llength $new] - 6]] \n]
		return -code error -errorcode $errorCode \
			-errorinfo $new $msg
	    } else {
		return -code $code $msg
	    }
	}
    }
    if {([info level] == 1) && ([info script] == "") \
	    && [info exists tcl_interactive] && $tcl_interactive} {
	if ![info exists auto_noexec] {
	    set new [auto_execok $name]
	    if {$new != ""} {
		set errorCode $savedErrorCode
		set errorInfo $savedErrorInfo
		set redir ""
		if {[info commands console] == ""} {
		    set redir ">&@stdout <@stdin"
		}
		return [uplevel exec $redir $new [lrange $args 1 end]]
	    }
	}
	set errorCode $savedErrorCode
	set errorInfo $savedErrorInfo
	if {$name == "!!"} {
#	    return [uplevel {history redo}]
	    return -code error "!! is disabled until history is fixed in Tcl8.0"
	}
	if [regexp {^!(.+)$} $name dummy event] {
	    return [uplevel [list history redo $event]]
	}
	if [regexp {^\^([^^]*)\^([^^]*)\^?$} $name dummy old new] {
	    return [uplevel [list history substitute $old $new]]
	}

	set ret [catch {set cmds [info commands $name*]} msg]
	if {[string compare $name "::"] == 0} {
	    set name ""
	}
	if {$ret != 0} {
	    return -code $ret -errorcode $errorCode \
		"error in unknown while checking if \"$name\" is a unique command abbreviation: $msg"
	}
	if {[llength $cmds] == 1} {
	    return [uplevel [lreplace $args 0 0 $cmds]]
	}
	if {[llength $cmds] != 0} {
	    if {$name == ""} {
		return -code error "empty command name \"\""
	    } else {
		return -code error \
			"ambiguous command name \"$name\": [lsort $cmds]"
	    }
	}
    }
    return -code error "invalid command name \"$name\""
}

namespace import -force rcx::*

# For testing
# set rcx::debug_level 1

# For automatic connection
# connect
