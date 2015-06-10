# Contributing Guidelines

Issues, pull requests, and other contributions are welcomed!

A few tips:

## Is `OSVR-FSM9` the right repository?

The OSVR system consists of a number of repositories, many of which are in the [OSVR organization on GitHub][osvr-org].

- If you've got an issue to report, a bug fix, or a feature addition to the OSVR-specific plugin code: **yes**, you're in the right spot!
- If you have an issue or contribution related to the data being retrieved through the FSM-9 driver: **you're close** - you'll either want to file an issue on [VRPN]. If you're not sure which, just file a support ticket and we'll help you out - see below.
- If you have a hardware or software issue related to OSVR but aren't sure exactly where it fits: **let us know with a support ticket at <http://support.osvr.com>**

[osvr-org]: https://github.com/osvr
[VRPN]: https://github.com/vrpn/vrpn

## Getting ready

When making pull requests, please fork the project and create a topic branch off of the `master` branch.
(This is what GitHub does by default if you start editing with your web browser.)

When developing, make small commits that are nevertheless "whole": small enough to review, but each containing a logical single change in its entirety.
(If you don't understand what we mean by this, that's OK, we'll work it out.)

It's OK to rebase your topic branch to make the history more clear.
Avoid merging from master into your topic branch: if you need a change from master, rebase; otherwise, try to keep topic branches short-lived enough that we can get your code in to the mainline before much else changes!

Try to develop code that is portable (not particularly tied to a single operating system/compiler/etc) - OSVR runs on a number of platforms, and while we don't expect you to have all of them to test on, it's good to keep in mind. Our continuous integration server will be able to help with this.

If you're adding something reasonably testable, please also add a test.
If you're touching code that already has tests, make sure they didn't break.

This repo follows the same code style guidelines as OSVR-Core.
The main points are to match code surrounding what you're edited, and to be sure to use `clang-format`.
These help ensure that your changes are not artificially large because of whitespace, etc, that it's easy to review your changes, and that your code will be maintainable in the future.

## License

No formal copyright assignment is required. If you're adding a new file, make sure it has the appropriate license header. Any contributions intentionally sent to the project are considered to be offered under the license of the project they're sent to.

