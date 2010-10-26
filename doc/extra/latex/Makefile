all: Move3Ddoc.pdf
	evince Move3Ddoc.pdf &

Move3Ddoc.pdf: Move3Ddoc.ps
	dvipdf Move3Ddoc.dvi

Move3Ddoc.ps: Move3Ddoc.dvi
	dvips -o Move3Ddoc.ps Move3Ddoc.dvi

Move3Ddoc.dvi: Move3Ddoc.tex
	latex Move3Ddoc
	bibtex Move3Ddoc
	latex Move3Ddoc
	latex Move3Ddoc

clean:
	# Destruction des rendus
	rm -rf Move3Ddoc.dvi Move3Ddoc.ps Move3Ddoc.pdf
	# Destruction des fichiers temporaires
	rm -f *.log *.aux *.toc *.blg *.bbl
