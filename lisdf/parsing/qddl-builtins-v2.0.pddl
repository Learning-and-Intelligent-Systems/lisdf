(define
  (domain lispddl-builtins)
  (:types
    qr::object
    qr::value
    qr::link  - qr::object
    qr::joint - qr::object
    qr::chain - qr::object
    qr::frame - qr::object
    qr::body  - qr::object
    qr::joint-conf - qr::value
    qr::chain-conf - qr::value
    qr::color      - qr::value  ; (r, g, b, a)

    qr::pose - qr::value  ; (x, y, z, roll, pitch, yaw)
  )

  (:object-types
    ; a built-in "world-type". Used to create the world frame.
    (qr::world-type   "")
    (qrgeom::box-type "")
  )

  (:predicates
    ; predicates for downstream parsers that do not support typing.
    (is-qr-link  ?link)
    (is-qr-joint ?joint)
    (is-qr-chain ?chain)
    (is-qr-frame ?frame)
    (is-qr-body  ?body)
    (is-qr-joint-conf ?pose)
    (is-qr-chain-conf ?pose)
    (is-qr-body-pose  ?pose)

    ; ?body: a free body
    ; ?pose: (x, y, z, roll, pitch, yaw)
    (body-pose ?body - qr::body ?pose - qr::pose)

    ; ?body: a body
    ; ?scale: a single valuekj
    (body-scale ?body - qr::body ?scale - qr::value)

    ; ?parent-frame: an object::frame_name of the parent
    ; ?child-frame: an object::frame_name of the child
    ; ?X_PC: the pose of the child relative to the parent, in (x, y, z, roll, pitch, yaw)
    (weld ?parent-frame - qr::frame ?child-frame - qr::frame ?X_PC - qr::pose)

    ; ?x: a joint of a robot
    ; ?c: a value
    (joint-conf ?x - qr::joint ?c - qr::joint-conf)

    ; ?x: a kinematic chain
    ; ?c: a list of values
      (chain-conf ?x - qr::chain ?c - qr::chain-conf)

    ; ?x: a body
    ; ?y: a link of the robot
    ; ?g: a grasp, specified as a transform from ?y to ?x, in (x, y, z, roll, pitch, yaw)
    (holding ?x - qr::body ?y - qr::link ?g - qr::pose)

    ; ?x: the name of the body
    ; ?size: the size of the box, as a tuple of (length_x, length_y, length_z)
    (qrgeom::box-shape ?x - qr::body ?size - qr::value)

    ; ?x: the name of the body
    ; ?color: the color of the box, as a tuple of (r, g, b, a)
    (qrgeom::box-color ?x - qr::body ?color - qr::color)
  )
)
