# Task Type

1. CartesianTask
3. ContactTask
4. SurfaceContact
5. VertexContact
6. PosturalTask
7. JointLimitsTask
8. RegularizationTask
9. RollingTask
10. ZmpTask

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
		type:Postural (tracking joint_ref keeping initial position)
