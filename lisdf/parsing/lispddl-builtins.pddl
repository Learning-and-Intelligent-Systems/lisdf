(define
    (domain lispddl-builtins)
    (:types
        sdf::link
        sdf::joint
        sdf::group
        sdf::model
        sdf::link-pose
        sdf::joint-pose
        sdf::group-pose
        sdf::model-pose
    )
    (:predicates
        ; predicates for downstream parsers that do not support typing.
        (sdf::is-link ?link)
        (sdf::is-joint ?joint)
        (sdf::is-group ?group)
        (sdf::is-model ?model)
        (sdf::is-link-pose ?pose)
        (sdf::is-joint-pose ?pose)
        (sdf::is-group-pose ?pose)
        (sdf::is-model-pose ?pose)

        ; specify poses for link, joint, group, and model.
        (sdf::at-link-pose ?link - sdf::link ?pose - sdf::link-pose)
        (sdf::at-joint-pose ?joint - sdf::joint ?pose - sdf::joint-pose)
        (sdf::at-group-pose ?group - sdf::group ?pose - sdf::group-pose)
        (sdf::at-model-pose ?model - sdf::model ?pose - sdf::model-pose)
    )
)
