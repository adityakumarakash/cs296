

#programs
ECHO=/bin/echo
CAT=cat


INSTALL_ROOT = .

.PHONY: all setup $(OBJS) exe data plots report doc data install

all : 
	@echo "This is installed folder \n Use make report to generate project report \n To run simulation execute ./mybins/cs296_17_exe \n To see documentation do gnome-open ./doc/html/index.html"	

cleandat:
	@rm -rf *.dat
clean:
	@rm -f $(BINDIR)/* $(OBJDIR)/* $(LIBSDIR)/*
	@rm -rf ./plots 
	@rm -f *.log *.log.bak
	@rm -rf ./doc/*.pdf ./doc/*.log ./doc/*.aux ./doc/*.html ./doc/*.blg ./doc/*.bbl ./doc/*.dvi
	@rm -rf  *.out
	@rm -rf ./data/g* ./data/pnp_austen_cs296.txt
	@rm -rf *.log  0* 1* 2* 3* 4* 5* 6* 7* 8* 9*
	@rm -rf *.tar.gz

distclean1: clean 
	@rm -rf $(EXTERNAL_ROOT)/src/Box2D $(EXTERNAL_ROOT)/lib $(EXTERNAL_ROOT)/lib/Box2D $(EXTERNAL_ROOT)/include/Box2D
	@rm -rf $(BINDIR) $(OBJDIR) $(LIBSDIR)
	@rm -rf $(PROJECT_ROOT)/plots 
	@rm -rf $(PROJECT_ROOT)/data
	
distclean:cleandat distclean1
plot:
	@mkdir -p plots
	@ipython ./scripts/g17_gen_plots.py

data:
	@mkdir -p data
	@python3 ./scripts/g17_gen_csv.py

report:
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


