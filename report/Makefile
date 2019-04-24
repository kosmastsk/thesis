MAIN=thesis

all:$(MAIN).tex
	xelatex -interaction=nonstopmode $(MAIN).tex

bib:$(MAIN).tex
	bibtex $(MAIN)

clean:
	rm -f *~ *.o *.out *.loa *.bbl *.blg *.toc *.lof *.lot *.aux *.dvi *.log *.pdf sections/*.aux sections/*log

