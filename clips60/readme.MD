# The History of CLIPS

The origins of the C Language Integrated Production System (CLIPS) date back to 1984 at
NASA’s Johnson Space Center. At this time, the Artificial Intelligence Section (now the
Software Technology Branch) had developed over a dozen prototype expert systems applications
using state-of-the-art hardware and software. However, despite extensive demonstrations of the
potential of expert systems, few of these applications were put into regular use. This failure to
provide expert systems technology within NASA’s operational computing constraints could
largely be traced to the use of LISP as the base language for nearly all expert system software
tools at that time. In particular, three problems hindered the use of LISP based expert system
tools within NASA: the low availability of LISP on a wide variety of conventional computers,
the high cost of state-of-the-art LISP tools and hardware, and the poor integration of LISP with
other languages (making embedded applications difficult).
The Artificial Intelligence Section felt that the use of a conventional language, such as C, would
eliminate most of these problems, and initially looked to the expert system tool vendors to
provide an expert system tool written using a conventional language. Although a number of tool
vendors started converting their tools to run in C, the cost of each tool was still very high, most
were restricted to a small variety of computers, and the projected availability times were
discouraging. To meet all of its needs in a timely and cost effective manner, it became evident
that the Artificial Intelligence Section would have to develop its own C based expert system tool.
The prototype version of CLIPS was developed in the spring of 1985 in a little over two months.
Particular attention was given to making the tool compatible with expert systems under
development at that time by the Artificial Intelligence Section. Thus, the syntax of CLIPS was
made to very closely resemble the syntax of a subset of the ART expert system tool developed by
Inference Corporation. Although originally modelled from ART, CLIPS was developed entirely
without assistance from Inference or access to the ART source code.
The original intent of the prototype was to gain useful insight and knowledge about the
construction of expert system tools and to lay the groundwork for the construction of a fully
usable tool. The CLIPS prototype had numerous shortcomings, however, it demonstrated the
feasibility of the project concept. After additional development, it became apparent that sufficient
enhancements to the prototype would produce a low cost expert system tool that would be ideal
for the purposes of training. Another year of development and internal use went into CLIPS
improving its portability, performance, and functionality. A reference manual and user’s guide
were written during this time. The first release of CLIPS to groups outside of NASA, version 3.0,
occurred in the summer of 1986.

Further enhancements transformed CLIPS from a training tool into a tool useful for the
development and delivery of expert systems as well. Versions 4.0 and 4.1 of CLIPS, released
respectively in the summer and fall of 1987, featured greatly improved performance, external
language integration, and delivery capabilities. Version 4.2 of CLIPS, released in the summer of
1988, was a complete rewrite of CLIPS for code modularity. Also included with this release were
an architecture manual providing a detailed description of the CLIPS software architecture and a
utility program for aiding in the verification and validation of rule-based programs. Version 4.3
of CLIPS, released in the summer of 1989, added still more functionality.
Originally, the primary representation methodology in CLIPS was a forward chaining rule lan-
guage based on the Rete algorithm (hence the Production System part of the CLIPS acronym).
Version 5.0 of CLIPS, released in the spring of 1991, introduced two new programming
paradigms: procedural programming (as found in languages such as C and Ada) and
object-oriented programming (as found in languages such as the Common Lisp Object System
and Smalltalk). The object-oriented programming language provided within CLIPS is called the
CLIPS Object-Oriented Language (COOL). Version 5.1 of CLIPS, released in the fall of 1991,
was primarily a software maintenance upgrade required to support the newly developed and/or
enhanced X Window, MS-DOS, and Macintosh interfaces.

Because of its portability, extensibility, capabilities, and low-cost, CLIPS has received
widespread acceptance throughout the government, industry, and academia. The development of
CLIPS has helped to improve the ability to deliver expert system technology throughout the
public and private sectors for a wide range of applications and diverse computing environments.
CLIPS is being used by over 4,000 users throughout the public and private community including:
all NASA sites and branches of the military, numerous federal bureaus, government contractors,
universities, and many private companies. CLIPS is available at a nominal cost through
COSMIC, the NASA software distribution center (for more on COSMIC, see appendix A of the
Basic Programming Guide).

# CLIPS Version 6.0

Version 6.0 of CLIPS contains five major enhancements. First, instances of user-defined classes
in COOL can be pattern-matched on the left-hand side of rules. Second, CLIPS now contains
considerable support for knowledge based systems software engineering. Support is now
provided for building modular systems and many of the features previously available in CRSV
are now directly supported in CLIPS (such as constraint consistency among uses of the same
variable). Third, deftemplates can now have more than one multifield slot. Fourth, it is now
possible to nest other conditional elements within a not conditional element and two new
conditional elements, exists and forall , are supported. Fifth, a Windows 3.1 CLIPS interface is
now available for PC compatible computers. In addition, an MS-DOS 386 version of CLIPS is
available which can use extended memory. For a detailed listing of differences between versions
5.1 and 6.0 of CLIPS, refer to appendix C of the Basic Programming Guide and appendix C of
the Advanced Programming Guide.
