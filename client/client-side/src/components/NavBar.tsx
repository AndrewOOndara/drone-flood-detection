// components/Navbar.tsx

"use client"

import { Box, Flex, HStack, IconButton, Link, Button, useDisclosure, VStack, Image } from "@chakra-ui/react";
import { useRouter } from "next/navigation";

const links = [
  { name: "Home", href: "/" },
  { name: "About", href: "/about" },
  { name: "Admin", href: "/admin" },
  { name: "Contact", href: "/contact" },
];

const Navbar = () => {
  const { isOpen, onOpen, onClose } = useDisclosure();
  const router = useRouter();

  return (
    <Box bg="white" px={4} py={2} pt={30} color="black">
      <Flex h={16} alignItems="center" justifyContent="space-between">
        {/* Logo on the left */}
        <Box fontWeight="bold" fontSize="lg" cursor="pointer" onClick={() => router.push("/")}>
          <Image src="/logo.png" alt="Logo" height="100px" />
        </Box>

        {/* Desktop Navigation */}
        <HStack spacing={8} alignItems="center" display={{ base: "none", md: "flex" }}>
          {links.map((link) => (
            <Link key={link.name} href={link.href} px={2} py={1} rounded="md" _hover={{ bg: "teal.700" }}>
              {link.name}
            </Link>
          ))}
        </HStack>

      </Flex>
    </Box>
  );
};

export default Navbar;
