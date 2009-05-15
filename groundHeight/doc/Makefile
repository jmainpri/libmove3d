.PHONY: doc clean

doc: package.dox
	doxygen $<

package.dox: package.dox.m4
	m4 $< > $@

clean:
	rm -rf html
	rm -f groundHeight.doxytag
	rm -f package.dox
