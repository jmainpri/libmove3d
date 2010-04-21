// author: Romain Iehl <riehl@laas.fr>

#ifndef VECTOR_OPERATIONS_HPP_INCLUDED
#define VECTOR_OPERATIONS_HPP_INCLUDED

template <typename T>
std::vector<T> selectNewElements(std::vector<T>& elements,
				 std::vector<T>& oldElements)
{
  std::vector<T> newElements;
#if __GNUC__ == 4 && __GNUC_MINOR__ >= 5
  // lambda-based
  // std::for_each(element.begin(), elements.end(), 
  // 		 [oldElements, newElements](element_t* j)
  // 		 { 
  // 		   if(std::find(oldElements.begin(), oldElements.end(), elements[i]) == oldElements.end()) {
  // 		     newElements.push_back(elements[i]);
  // 		     oldElements.push_back(elements[i]);
  // 		   }
  // 		 });
  // initializer list based
  for(auto T : elements)
  {
    if(std::find(oldElements.begin(), oldElements.end(), elements[i]) == oldElements.end())
    {
      newElements.push_back(elements[i]);
      oldElements.push_back(elements[i]);
    }
  }
#else
  for(uint i(0); i < elements.size(); i++)
  {
    if(std::find(oldElements.begin(), oldElements.end(), elements[i]) == oldElements.end())
    {
      newElements.push_back(elements[i]);
      oldElements.push_back(elements[i]);
    }
  }
#endif
  return(newElements);
}

#endif
