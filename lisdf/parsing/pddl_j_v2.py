import os.path as osp
from typing import Optional, Tuple

import numpy as np
from lark import Lark, Transformer, Tree, v_args

import lisdf.components as C
from lisdf.components.base import NAME_SCOPE_SEP
from lisdf.parsing.pddl_j import LISPDDLParser, PDDLVisitor

inline_args = v_args(inline=True)


class LISPDDLParserV2(LISPDDLParser):
    """LISPDDLParserV2 adds support for

    - a new built set of predecates, such as body pose, scale, and color.
    - a new section in domain files for importing SDF/URDF files.
    """

    grammar_file = osp.join(osp.dirname(__file__), "lispddl-v2.0.grammar")
    builtins_file = osp.join(osp.dirname(__file__), "lispddl-builtins-v2.0.pddl")

    def __init__(self, strict_sdf_name_checking: bool = False) -> None:
        """Initialize the parser.

        Args:
            strict_sdf_name_checking: if true, the parser will check if sdf component names (e.g., joints and links) are valid.
        """

        super().__init__()
        self.strict_sdf_name_checking = strict_sdf_name_checking

        if self.strict_sdf_name_checking:
            raise NotImplementedError("strict_sdf_name_checking is not implemented yet.")

    def transform(self, domain_tree: Tree, problem_tree: Tree) -> Tuple[C.PDDLDomain, C.PDDLProblem]:
        domain_tree = domain_tree.children[0]
        problem_tree = problem_tree.children[0]

        domain = C.PDDLDomain("")
        transformer = PDDLVisitorV2(domain, C.PDDLProblem("", domain))
        transformer.set_mode("extend")
        builtins = self.load(type(self).builtins_file).children[0]
        transformer.transform(builtins)
        transformer.set_mode("domain")
        transformer.transform(domain_tree)
        transformer.set_mode("problem")
        transformer.transform(problem_tree)
        return transformer.domain, transformer.problem


class PDDLVisitorV2(PDDLVisitor):
    def __init__(self, domain: C.PDDLDomain, problem: C.PDDLProblem) -> None:
        super().__init__(None, domain, problem)

    @inline_args
    def object_type_definition(self, object_type, url_string):
        object_type = object_type.children[0][1]
        if object_type in self.domain.types or object_type in self.domain.object_types:
            raise KeyError(f'Type name {object_type} already exists.')
        self.domain.object_types[object_type] = C.PDDLObjectType(object_type, url_string)

    @inline_args
    def constant(self, name):
        name = name.value

        model_name = name
        component_name = None
        if NAME_SCOPE_SEP in name:
            model_name, component_name = name.split(NAME_SCOPE_SEP)

        type = None
        if name in self.problem.objects:
            type = self.problem.objects[name].type
        elif model_name in self.problem.objects:
            type = self.problem.objects[model_name].type

        sdf_object = None
        if type is not None and component_name is not None:
            if isinstance(type, C.PDDLObjectType):
                sdf_object = C.PDDLSDFObject(model_name, component_name, None)
            else:
                assert component_name is None, f"Component name is not allowed for object types, got {name}{NAME_SCOPE_SEP}{component_name}."

        return C.PDDLObject(name, type, sdf_object=sdf_object)

    @inline_args
    def typedconstant(self, constant, typename):  # used in :init definition.
        name = constant.name
        typename = typename.children[0][1]

        model_name = name
        component_name = None
        if NAME_SCOPE_SEP in name:
            model_name, component_name = name.split(NAME_SCOPE_SEP)

        type = None
        if typename in self.domain.object_types:
            type = self.domain.object_types[typename]
        elif typename in self.domain.types:
            type = self.domain.types[typename]
        else:
            raise KeyError(f"Type {typename} is not defined.")

        sdf_object = None
        if isinstance(type, C.PDDLObjectType):
            sdf_object = C.PDDLSDFObject(model_name, component_name, None)

        return C.PDDLObject(name, type, sdf_object=sdf_object)


default_pddl_parser_v2 = LISPDDLParserV2()


def load_pddl(domain_file: str, problem_file: str) -> Tuple[C.PDDLDomain, C.PDDLProblem]:

    return default_pddl_parser_v2.transform(
        default_pddl_parser_v2.load(domain_file),
        default_pddl_parser_v2.load(problem_file),
    )


def load_pddl_string(domain_string: str, problem_string: str) -> Tuple[C.PDDLDomain, C.PDDLProblem]:
    return default_pddl_parser_v2.transform(
        default_pddl_parser_v2.loads(domain_string),
        default_pddl_parser_v2.loads(problem_string),
    )

