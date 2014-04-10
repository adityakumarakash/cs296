

#programs
ECHO=/bin/echo
CAT=cat


INSTALL_ROOT = .

.PHONY: all setup data plots report doc data install

all : 
	@echo "This is installed folder \n Use make report to generate project report \n To run simulation execute ./mybins/cs296_17_exe \n To see documentation do gnome-open ./doc/html/index.html"	

cleandat:
	@rm -rf *.dat
clean:
	@rm -rf plots data
	@rm -f *.log *.log.bak
	@rm -rf ./doc/*.pdf ./doc/*.log ./doc/*.aux ./doc/*.html ./doc/*.blg ./doc/*.bbl ./doc/*.dvi
	@rm -rf  *.out
	@rm -rf ./data/g*
	@rm -rf *.log  0* 1* 2* 3* 4* 5* 6* 7* 8* 9*
	@rm -rf *.tar.gz

distclean1: clean 
	
	
distclean:cleandat distclean1
plot:
	@if test -e data/g17_data_01.csv; \
		then echo ""; \
		else mkdir -p data\
		&&python3 ./scripts/g17_gen_csv.py; \
	fi;
	@mkdir -p plots
	@ipython ./scripts/g17_gen_plots.py

data:
	@mkdir -p data
	@python3 ./scripts/g17_gen_csv.py

report:
	@if test -e ./plots/g17_plot05.png;\
		then echo "plots already present";\
		else make plot ; \
	fi;
	@python3 ./scripts/g17_gen_html.py
	@gnome-open ./doc/g17_report.html
	
	@cd doc \
	&& pdflatex ./cs296_report_17.tex \
	&& bibtex ./cs296_report_17 \
	&& bibtex ./cs296_report_17 \
	&& pdflatex ./cs296_report_17.tex \
	&& bibtex ./cs296_report_17 \
	&& pdflatex ./cs296_report_17.tex \
	&& cd ..
	@rm -rf ./doc/*.log ./doc/*.aux  ./doc/*.blg ./doc/*.bbl ./doc/*.dvi


