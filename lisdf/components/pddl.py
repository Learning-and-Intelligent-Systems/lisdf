from abc import ABC
from dataclasses import dataclass, field
from typing import Any, ClassVar, Dict, List, Optional, Union

import numpy as np

from lisdf.components.base import NAME_SCOPE_SEP, StringifyContext
from lisdf.utils.printing import indent_text

# Not used for now. In the future, we want to implement more complex expressions.
PDDL_NOT = "not"
PDDL_AND = "and"
PDDL_OR = "or"
PDDL_IMPLIES = "=>"
PDDL_FORALL = "forall"
PDDL_EXISTS = "exists"
PDDL_EQUALS = "="
PDDL_LESS = "<"
PDDL_GREATER = ">"
PDDL_LESS_EQUALS = "<="
PDDL_GREATER_EQUALS = ">="
PDDL_NOT_EQUALS = "!="

PDDL_SCOPE_SEP = "::"


def set_pddl_scope_sep(sep: str) -> None:
    global PDDL_SCOPE_SEP
    PDDL_SCOPE_SEP = sep


class PDDLStringConfigurable(ABC):
    DEFAULT_PDDL_STRINGIFY_OPTIONS: ClassVar[Dict[str, Any]] = {
        "use_types": True,
    }

    def to_pddl(self, ctx: Optional[StringifyContext] = None, **kwargs) -> str:
        if ctx is None:
            for k, v in PDDLStringConfigurable.DEFAULT_PDDL_STRINGIFY_OPTIONS.items():
                kwargs.setdefault(k, v)
            ctx = StringifyContext(**kwargs)
        return self._to_pddl(ctx)

    def _to_pddl(self, ctx: StringifyContext) -> str:
        raise NotImplementedError()


@dataclass
class PDDLType(PDDLStringConfigurable):
    identifier: str
    parent: Optional["PDDLType"] = None
    scope: Optional[str] = None

    def __init__(self, identifier: str, parent: Optional["PDDLType"] = None) -> None:
        if PDDL_SCOPE_SEP in identifier:
            self.scope, self.identifier = identifier.split(PDDL_SCOPE_SEP)
        else:
            self.identifier = identifier
            self.scope = None
        self.parent = parent

    @property
    def pddl_name(self) -> str:
        return (
            self.scope + PDDL_SCOPE_SEP + self.identifier
            if self.scope
            else self.identifier
        )

    def _to_pddl(self, ctx: StringifyContext) -> str:
        if self.parent is None:
            return self.pddl_name
        else:
            return f"{self.pddl_name} - {self.parent.to_pddl(ctx)}"


@dataclass
class PDDLVariable(PDDLStringConfigurable):
    name: str
    type: Optional[PDDLType] = None

    @property
    def pddl_name(self) -> str:
        return self.name

    def _to_pddl(self, ctx: StringifyContext) -> str:
        if ctx.options["use_types"] and self.type is not None:
            return f"({self.name} - {self.type.to_pddl(ctx)})"
        else:
            return self.name


@dataclass
class PDDLPredicate(PDDLStringConfigurable):
    name: str
    arguments: List[PDDLVariable] = field(default_factory=list)
    return_type: PDDLType = PDDLType("bool")
    scope_name: Optional[str] = None

    def __init__(
        self,
        name: str,
        arguments: Optional[List[PDDLVariable]] = None,
        return_type: Optional[PDDLType] = None,
    ):
        if PDDL_SCOPE_SEP in name:
            self.scope_name, self.name = name.split(PDDL_SCOPE_SEP)
        else:
            self.name = name
            self.scope_name = None

        self.arguments = arguments if arguments is not None else []
        self.return_type = return_type if return_type is not None else PDDLType("bool")

    @property
    def pddl_name(self) -> str:
        return (
            self.scope_name + PDDL_SCOPE_SEP + self.name
            if self.scope_name
            else self.name
        )

    def _to_pddl(self, ctx: StringifyContext) -> str:
        # TODO (Jiayuan Mao@04/04): warning about non-boolean return type or
        # automatically convert to boolean.

        arguments_str = " ".join([v.to_pddl(ctx) for v in self.arguments])
        return f"({self.pddl_name} {arguments_str})"


@dataclass
class PDDLOperator(PDDLStringConfigurable):
    name: str
    arguments: List[PDDLVariable] = field(default_factory=list)
    preconditions: List[PDDLPredicate] = field(default_factory=list)
    add_effects: List[PDDLPredicate] = field(default_factory=list)
    del_effects: List[PDDLPredicate] = field(default_factory=list)

    def _to_pddl(self, ctx: StringifyContext) -> str:
        raise NotImplementedError()


# TODO (Jiayuan Mao@04/04): add support for advanced PDDL operators.
# a.k.a. PDSketch :)


@dataclass
class PDDLObject(PDDLStringConfigurable):
    name: str
    type: Optional[PDDLType] = None
    sdf_object: Optional["PDDLSDFObject"] = None

    @property
    def pddl_name(self) -> str:
        return self.name

    def _to_pddl(self, ctx: StringifyContext) -> str:
        if ctx.options["use_types"] and self.type is not None:
            return f"{self.name} - {self.type.to_pddl(ctx)}"
        else:
            return self.name


@dataclass(frozen=True)
class PDDLSDFObject(PDDLStringConfigurable):
    model_name: str
    name: Optional[str]
    sdf_type: PDDLType

    @property
    def pddl_name(self) -> str:
        if self.name is None:
            return self.model_name
        else:
            if NAME_SCOPE_SEP is None:
                return self.name
            return self.model_name + NAME_SCOPE_SEP + self.name

    def _to_pddl(self, ctx: StringifyContext) -> str:
        return self.pddl_name

    def to_pddl_object(self) -> PDDLObject:
        return PDDLObject(self.pddl_name, self.sdf_type, sdf_object=self)


@dataclass
class PDDLValue(PDDLStringConfigurable, ABC):
    pass


@dataclass
class PDDLLiteral(PDDLValue):
    value: Union[bool, int, float, str]

    def _to_pddl(self, ctx: StringifyContext) -> str:
        return str(self.value)


@dataclass
class PDDLVectorValue(PDDLValue):
    value: np.ndarray

    def _to_pddl(self, ctx: StringifyContext) -> str:
        return str(self.value.tolist())


@dataclass
class PDDLNamedValue(PDDLStringConfigurable):
    name: str
    value: PDDLValue

    def _to_pddl(self, ctx: StringifyContext) -> str:
        return f"{self.name}={self.value.to_pddl(ctx)}"


@dataclass
class PDDLProposition(PDDLStringConfigurable):
    predicate: PDDLPredicate
    arguments: List[PDDLObject] = field(default_factory=list)

    def _to_pddl(self, ctx: StringifyContext) -> str:
        arguments_str = " ".join(
            a.pddl_name if isinstance(a, PDDLObject) else a.to_pddl(ctx)
            for a in self.arguments
        )
        return f"({self.predicate.pddl_name} {arguments_str})"


@dataclass
class PDDLDomain(PDDLStringConfigurable):
    name: str
    types: Dict[str, PDDLType] = field(default_factory=dict)
    constants: Dict[str, PDDLObject] = field(default_factory=dict)
    predicates: Dict[str, PDDLPredicate] = field(default_factory=dict)
    operators: Dict[str, PDDLOperator] = field(default_factory=dict)

    def _to_pddl(self, ctx: StringifyContext) -> str:
        fmt = f"(define (domain {self.name})\n"
        fmt += "  (:requirements :strips :typing)\n"
        fmt += "  (:types\n"
        fmt += (
            "    "
            + indent_text("\n".join([t.to_pddl(ctx) for t in self.types.values()]), 2)
            + "\n"
        )
        fmt += "  )\n"
        fmt += "  (:constants\n"
        fmt += (
            "    "
            + indent_text(
                "\n".join([c.to_pddl(ctx) for c in self.constants.values()]), 2
            )
            + "\n"
        )
        fmt += "  )\n"
        fmt += "  (:predicates\n"
        fmt += (
            "    "
            + indent_text(
                "\n".join([p.to_pddl(ctx) for p in self.predicates.values()]), 2
            )
            + "\n"
        )
        fmt += "  )\n"
        fmt += ")\n"
        return fmt


@dataclass
class PDDLProblem(PDDLStringConfigurable):
    name: str
    domain: PDDLDomain
    objects: Dict[str, PDDLObject] = field(default_factory=dict)
    init: List[PDDLProposition] = field(default_factory=list)
    conjunctive_goal: List[PDDLPredicate] = field(default_factory=list)

    def _to_pddl(self, ctx: StringifyContext) -> str:
        fmt = f"(define (problem {self.name})\n"
        fmt += f"  (:domain {self.domain.name})\n" if self.domain is not None else ""
        fmt += "  (:objects\n"
        fmt += (
            "    "
            + indent_text("\n".join([o.to_pddl(ctx) for o in self.objects.values()]), 2)
            + "\n"
        )
        fmt += "  )\n"
        fmt += "  (:init\n"
        fmt += (
            "    "
            + indent_text("\n".join([p.to_pddl(ctx) for p in self.init]), 2)
            + "\n"
        )
        fmt += "  )\n"
        fmt += "  (:goal (and\n"
        fmt += (
            "    "
            + indent_text("\n".join([p.to_pddl(ctx) for p in self.conjunctive_goal]), 2)
            + "\n"
        )
        fmt += "  ))\n"
        fmt += ")\n"
        return fmt


@dataclass
class PDDLFunctionCall(PDDLStringConfigurable):
    """This is currently only used as a temporary data structure for
    function applications in PDDL. Since we currently only support the most
    basic strips functionality, the op_name can only be and or not. But in the
    future we might want to support more complex function applications. Then we
    should split this class to a base class and multiple derived classes.
    """

    op_name: str
    arguments: List[Union["PDDLFunctionCall", PDDLVariable, PDDLValue]] = field(
        default_factory=list
    )

    def _to_pddl(self, ctx: StringifyContext) -> str:
        arguments_str = " ".join(a.to_pddl(ctx) for a in self.arguments)
        return f"({self.op_name} {arguments_str})"
