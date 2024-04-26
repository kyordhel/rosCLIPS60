
(defglobal ?*id-mvn-pln* = 0)


(deftemplate on-top-of
	(slot upper)
	(slot lower)
)


(deftemplate goal (slot move)(slot on-top-of))


(deffacts initial-state
	(block A)
	(block B)
	(block C)
	(block D)
	(block E)
	(block F)
	(on-top-of (upper nothing)(lower A))
	(on-top-of (upper A)(lower B))
	(on-top-of (upper B)(lower C))
	(on-top-of (upper C)(lower floor))
	(on-top-of (upper nothing)(lower D))
	(on-top-of (upper D)(lower E))
	(on-top-of (upper E)(lower F))
	(on-top-of (upper F)(lower floor))
	;(goal (move C)(on-top-of E))
	(goal (move F)(on-top-of C))
	(ros-node "clips_out")
)


; It sends a message that clips is alive
(defrule clips-alive
	  (declare (salience 1000))
	  (ros-node ?node)
        =>
          (rospub ?node "Clips alive")
)


;it sends the command to the destination 
(defrule send-command-node
           ?fact1 <- (send-network ?destination ?arguments ?id)
        =>
          (retract ?fact1)
          (bind ?status (rospub ?destination ?arguments))
          (printout t "published to " ?destination ": " ?arguments crlf)
)


;it transforms the robot command
(defrule transform-send-robot
        ?fact1 <- (send-robot ?message)
	(ros-node ?node)
        =>
        (retract ?fact1)
        (assert (send-network ?node ?message ?*id-mvn-pln*))
        (bind ?*id-mvn-pln* (+ ?*id-mvn-pln* 1))
)


(defrule move-directly
	?goal <- (goal (move ?block1) (on-top-of ?block2))
	(block ?block1)
	(block ?block2)
	(on-top-of (upper nothing) (lower ?block1))
	?stack-1 <- (on-top-of (upper ?block1)(lower ?block3))
	?stack-2 <- (on-top-of (upper nothing)(lower ?block2))
	=>
	(retract ?goal ?stack-1 ?stack-2)
	(assert (on-top-of (upper ?block1)(lower ?block2))
       	  	(on-top-of (upper nothing)(lower ?block3)))
	(printout t ?block1 " moved on top of " ?block2 "." crlf)
	(bind ?command (str-cat "move block " ?block1 " on top of block" ?block2))
	(assert (send-robot ?command))
)


(defrule move-to-floor
	?goal <- (goal (move ?block1) (on-top-of floor))
	(block ?block1)
	(on-top-of (upper nothing) (lower ?block1))
	?stack <- (on-top-of (upper ?block1) (lower ?block2))	
	=>
	(retract ?goal ?stack)
	(assert (on-top-of (upper ?block1)(lower floor))
        	(on-top-of (upper nothing)(lower ?block2)))
	(printout t ?block1 " moved on top of floor. " crlf)
        (bind ?command (str-cat "move block " ?block1 " to the floor"))
        (assert (send-robot ?command))
)


(defrule clear-upper-block
	(goal (move ?block1))
	(block ?block1)
	(on-top-of (upper ?block2) (lower ?block1))
	(block ?block2)
	=>
	(assert (goal (move ?block2)(on-top-of floor)))
)


(defrule clear-lower-block
	(goal (on-top-of ?block1))
	(block ?block1)
	(on-top-of (upper ?block2) (lower ?block1))
	(block ?block2)
	=>
	(assert (goal (move ?block2)(on-top-of floor)))
)

