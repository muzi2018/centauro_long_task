# Task Type

1. CartesianTask
2. ContactTask
3. SurfaceContact
4. VertexContact
5. PosturalTask
6. JointLimitsTask
7. RegularizationTask
8. RollingTask
9. ZmpTask

constraints:
	-contact_1
		type: Contact
		subtask: [interaction_1,rolling_task_1]
			interaction_1:
				type: VertexContact (limite force value of contact_1)
			rolling_task_1:
				type: Rolling (move the wheel)
    	-contact_2
	-contact_3
	-contact_4

costs:
	-joint_regularization
		type: Regularization (penalt the joints' vel and acc for stability)
	-joint_posture
		type:Postural (keeping initial position)
