import os.path as osp
from typing import Optional, Union, Tuple

import numpy as np
from lark import Lark, Transformer, Tree, v_args

import lisdf.components.pddl as C
from lisdf.components.base import NAME_SCOPE_SEP

# lark.v_args
inline_args = v_args(inline=True)
DEBUG_LOG_COMPOSE = False


class QDDLParser(object):
    """QDDLParser adds support for

    - a new built set of predecates, such as body pose, scale, and color.
    - a new section in domain files for importing SDF/URDF files.
    """

    grammar_file = osp.join(osp.dirname(__file__), "qddl-v2.0.grammar")
    builtins_file = osp.join(osp.dirname(__file__), "qddl-builtins-v2.0.pddl")

    def __init__(self, strict_sdf_name_checking: bool = False) -> None:
        """Initialize the parser.

        Args:
            strict_sdf_name_checking: if true, the parser will check if sdf component names (e.g., joints and links) are valid.
        """

        with open(type(self).grammar_file) as f:
            self.lark = Lark(f, propagate_positions=True)
        self.strict_sdf_name_checking = strict_sdf_name_checking

        if self.strict_sdf_name_checking:
            raise NotImplementedError("strict_sdf_name_checking is not implemented yet.")

    def load(self, file):
        with open(file) as f:
            return self.lark.parse(f.read())

    def loads(self, string):
        return self.lark.parse(string)

    def transform(self, domain_tree: Tree, problem_tree: Tree) -> Tuple[C.PDDLDomain, C.PDDLProblem]:
        domain_tree = domain_tree.children[0]
        problem_tree = problem_tree.children[0]

        domain = C.PDDLDomain("")
        transformer = QDDLVisitor(domain, C.PDDLProblem("", domain))
        transformer.set_mode("extend")
        builtins = self.load(type(self).builtins_file).children[0]
        transformer.transform(builtins)
        transformer.set_mode("domain")
        transformer.transform(domain_tree)
        transformer.set_mode("problem")
        transformer.transform(problem_tree)
        return transformer.domain, transformer.problem


class QDDLVisitor(Transformer):
    def __init__(self, domain: C.PDDLDomain, problem: C.PDDLProblem) -> None:
        super().__init__()

        self.domain = domain
        self.problem = problem
        self.mode = "domain"

    def set_mode(self, mode: str) -> None:
        assert mode in ("domain", "problem", "extend")
        self.mode = mode

    @inline_args
    def definition_decl(self, definition_type, definition_name):
        if self.mode == "domain":
            assert definition_type.value == "domain"
            self.domain.name = definition_name.value
        elif self.mode == "problem":
            assert definition_type.value == "problem"
            self.problem.name = definition_name.value
        elif self.mode == "extend":
            assert definition_type.value == "domain"

    def type_definition(self, args):
        # Very ugly hack to handle multi-line definition in PDDL.
        # In PDDL, type definition can be separated by newline.
        # This kinds of breaks the parsing strategy that ignores all whitespaces.
        # More specifically, consider the following two definitions:
        # ```
        # (:types
        #   a
        #   b - a
        # )
        # ```
        # and
        # ```
        # (:types
        #   a b - a
        # )
        if args[-1].data == "parent_type_name":
            parent_line, parent_name = args[-1].children[0].children[0]
            args = args[:-1]
        else:
            parent_line, parent_name = -1, None
        for arg in args:
            arg_line, arg_name = arg.children[0]
            if arg_line == parent_line:
                parent_type = self.domain.types[parent_name]
                self.domain.types[arg_name] = C.PDDLType(arg_name, parent_type)
            else:
                self.domain.types[arg_name] = C.PDDLType(arg_name, None)

    @inline_args
    def constant_definition(self, constant):
        self.domain.constants[constant.name] = constant

    @inline_args
    def predicate_definition(self, name, *args):
        self.domain.predicates[name] = C.PDDLPredicate(name, args)

    @inline_args
    def object_definition(self, constant):
        self.problem.objects[constant.name] = constant

    @inline_args
    def init_definition_item(self, proposition):
        assert isinstance(proposition, C.PDDLProposition)
        self.problem.init.append(proposition)
        for arg in proposition.arguments:
            if isinstance(arg, C.PDDLObject):
                if (
                    arg.name not in self.problem.objects
                    and arg.name not in self.domain.constants
                    and arg.sdf_object is None
                ):
                    raise NameError(
                        "Unknown object: {} in {}.".format(
                            arg.name, proposition.to_pddl()
                        )
                    )

    @inline_args
    def goal_definition(self, goal):
        assert goal.op_name == "and"
        for g in goal.arguments:
            self.problem.conjunctive_goal.append(g)

    @inline_args
    def variable(self, name):
        return C.PDDLVariable(name.value)

    @inline_args
    def typedvariable(self, name, typename):
        # name is of type `PDDLVariable`.
        return C.PDDLVariable(name.name, self.domain.types[typename.children[0][1]])

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

    @inline_args
    def int(self, v):
        return C.PDDLLiteral(int(v.value))

    @inline_args
    def float(self, v):
        return C.PDDLLiteral(float(v.value))

    @inline_args
    def string(self, v):
        return C.PDDLLiteral(v.value[1:-1])

    @inline_args
    def list(self, *args):
        return C.PDDLVectorValue(np.array([arg.value for arg in args]))

    @inline_args
    def named_literal(self, name, value):
        return C.PDDLNamedValue(name.value, value)

    @inline_args
    def type_name(self, name):
        # propagate the "lineno" of the type definition up.
        return name.line, name.value

    @inline_args
    def function_name(self, name):
        return name.value

    @inline_args
    def method_name(self, feature_name, _, method_name):
        return feature_name.value + "::" + method_name.value

    @inline_args
    def function_call(self, name, *args):
        if name == "and":
            return C.PDDLFunctionCall(C.PDDL_AND, args)
        elif name == "or":
            return C.PDDLFunctionCall(C.PDDL_OR, args)
        elif name == "not":
            return C.PDDLFunctionCall(C.PDDL_NOT, args)
        elif name == "exists":
            return C.PDDLFunctionCall(C.PDDL_EXISTS, args)
        elif name == "forall":
            return C.PDDLFunctionCall(C.PDDL_FORALL, args)
        else:
            assert name in self.domain.predicates, 'Unknown predicate "{}"'.format(name)
            return C.PDDLProposition(self.domain.predicates[name], args)


default_qddl_parser = QDDLParser()


def load_qddl(domain_file: str, problem_file: str) -> Tuple[C.PDDLDomain, C.PDDLProblem]:
    return default_qddl_parser.transform(
        default_qddl_parser.load(domain_file),
        default_qddl_parser.load(problem_file),
    )


def load_qddl_string(domain_string: str, problem_string: str) -> Tuple[C.PDDLDomain, C.PDDLProblem]:
    return default_qddl_parser.transform(
        default_qddl_parser.loads(domain_string),
        default_qddl_parser.loads(problem_string),
    )


def parse_qddl_expression(domain: C.PDDLDomain, problem: Optional[C.PDDLProblem], expression_string: str) -> Union[C.PDDLFunctionCall, C.PDDLProposition]:
    transformer = QDDLVisitor(domain, problem if problem is not None else C.PDDLProblem("", domain))
    transformer.set_mode("problem")
    return transformer.transform(default_qddl_parser.loads(expression_string).children[0])

