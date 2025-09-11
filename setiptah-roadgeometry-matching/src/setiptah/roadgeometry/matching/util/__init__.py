from functools import cached_property


def singleton(factory):
    return factory()


def inner_class(cls):

    cls_dict = dict(cls.__dict__)
    cls_name = cls.__name__
    cls_bases = cls.__bases__
    # cls_globals = cls.__module__

    # Remove internals Python copies by default
    cls_dict.pop('__dict__', None)
    cls_dict.pop('__weakref__', None)

    @cached_property
    def inner(outer):
        cls_ = type(cls_name, cls_bases, dict(cls_dict))
        cls_.__outer__ = outer
        return cls_

    return inner
