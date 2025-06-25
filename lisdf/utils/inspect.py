#! /usr/bin/env python3

import argparse
from lisdf.components.pddl import PDDLObject
from lisdf.parsing.qddl import load_qddl


def main(args):
    if len(args.files) == 1:
        raise NotImplementedError('Inpsecting a single domain or problem file is not supported yet.')
    elif len(args.files) == 2:
        domain, problem = load_qddl(args.files[0], args.files[1])

        print(f'Problem Name: {problem.name}')
        print('Objects:')
        for k, v in problem.objects.items():
            print(' ', v)
        print('Initial Conditions:')
        for prop in problem.init:
            print(' ', prop.predicate.name, end='(')
            x_strings = list()
            for arg, x in zip(prop.predicate.arguments, prop.arguments):
                if isinstance(x, PDDLObject):
                    x_strings.append(f'{arg.name}={x.name}')
                else:
                    x_strings.append(f'{arg.name}={str(x)}')
            print(', '.join(x_strings), end=')\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs="+")
    main(parser.parse_args())

