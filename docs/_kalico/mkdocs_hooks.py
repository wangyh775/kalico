# Tool to customize conversion of markdown files during mkdocs site generation
import re
import logging
import sys
import os

# Ensure this module is importable for YAML !!python/name: tags
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from pymdownx.slugs import slugify as _pymdownx_slugify

# CJK-aware slugify that also strips leading/trailing hyphens
# (e.g. from emoji ⚠️ headings being stripped, leaving "-name")
_slugify_fn = _pymdownx_slugify(case="lower")


def custom_slugify(text, sep):
    return _slugify_fn(text, sep).strip("-")


def on_config(config, **kwargs):
    """Inject CJK-aware slugify into the toc extension."""
    config["mdx_configs"]["toc"]["slugify"] = custom_slugify

# This script translates some github specific markdown formatting to
# improve rendering with mkdocs.  The goal is for pages to render
# similarly on both github and the web site.  It has three main tasks:
# 1. Convert links outside of the docs directory (any reference
#    starting with "../") to an absolute link to the raw file on
#    github.
# 2. Convert a trailing backslash on a text line to a "<br>".
# 3. Remove leading spaces from top-level lists so that those lists
#    are rendered correctly.

logger = logging.getLogger("mkdocs.mkdocs_hooks.transform")


def transform(markdown: str, page, config, files):
    in_code_block = 0
    in_list = False
    lines = markdown.splitlines()
    for i in range(len(lines)):
        line_out = lines[i]
        in_code_block = (
            in_code_block + len(re.findall(r"\s*[`]{3,}", line_out))
        ) % 2
        if not in_code_block:
            line_out = line_out.replace(
                "](../", f"]({config['repo_url']}/blob/main/"
            )
            line_out = re.sub(r"\\\s*$", "<br>", line_out)
            # check that lists at level 0 are not indented
            # (no space before *|-|1.)
            if re.match(r"^[^-*0-9 ]", line_out):
                in_list = False
            elif re.match(r"^(\*|-|\d+\.) ", line_out):
                in_list = True
            if not in_list:
                line_out = re.sub(r"^\s+(\*|-|\d+\.) ", r"\1 ", line_out)
            if line_out != lines[i]:
                logger.debug(
                    (
                        f"[mkdocs_hooks] rewrite line {i+1}: "
                        f'"{lines[i]}" -> "{line_out}"'
                    )
                )
            lines[i] = line_out
    output = "\n".join(lines)
    return output
