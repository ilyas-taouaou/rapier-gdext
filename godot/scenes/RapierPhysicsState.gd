extends RapierPhysicsState

func _ready():
	await get_tree().physics_frame
	var box = BoxMesh.new()
	box.material = StandardMaterial3D.new()
	box.material.albedo_color = Color.RED
	var box_shape = BoxShape3D.new()
	while true:
		await get_tree().create_timer(0).timeout
		var body = RapierPhysicsRigidBody.new()
		body.global_position = Vector3(randf_range(-5, 5), randf_range(5, 10), randf_range(-5, 5))
		var collider = RapierPhysicsCollider.new()
		var mesh = MeshInstance3D.new()
		mesh.mesh = box
		collider.shape = box_shape
		body.add_child(mesh)
		body.add_child(collider)
		add_child(body)
