#
# TableBot
#
# Drive around a table top without falling off

# extra turn time to get away from the table edge
set turn_extra r10

begin task 0
sensor 1 reads touch
sensor 1 uses bool
sensor 3 reads touch
sensor 3 uses bool
wait 100

motor AC forward
motor AC power 5
motor AC on
start 1

jmp 0


begin task 1

# Wait until one or both sensors sees a table edge
test:
rset r0 sensor 1
rset r1 sensor 3
tbr {r0 == 1} left_or_both
tbr {r1 == 0} test

right:
# The right sensor went off
# Turn left until it is back on
motor A reverse
tbr {sensor 3 == 1}
wait $turn_extra
motor A forward
jmp test

left_or_both:
beep 1
tbr {r1 == 1} both

left:
# The left sensor went off, turn right until it is back on
motor C reverse
tbr {sensor 1 == 1}
wait $turn_extra
motor C forward
jmp test

both:
# Both sensors went off, turn around until both are back on
beep 2
# Maybe back up first?
# FIXME: choose a random direction to turn
motor A forward
motor C reverse
both2:
rset r0 sensor 1
rset r1 sensor 3
calc r0 + r1
tbr {r0 != 0} both2
wait $turn_extra
motor C forward
jmp test


end
